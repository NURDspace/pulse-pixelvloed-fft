#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <signal.h>
#include <pulse/simple.h>
#include <pulse/error.h>
#include <fftw3.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <time.h>
#include <sys/time.h>
#include <systemd/sd-daemon.h>

//gcc -Ofast test2.c -o test3 -lm -lpulse -lpulse-simple -lfftw3
//Tnx to https://gitlab.com/nitroxis/pasa/ for providing a good example of how to do this

#define COLS 128 
#define LINES 64 
#pragma pack(1)

struct Pixel {
    uint16_t y;
    uint16_t x;
    uint8_t r;
    uint8_t g;
    uint8_t b;	   
} pixel;  

struct Packet {
    uint16_t header;
    struct Pixel pixel[16];
} packet;

#define peakHoldTimeout 500
uint8_t peakHold[COLS];
long long peakHoldTime[COLS];

struct sigaction old_sigint;
volatile bool run;

int framesPerSecond = 10;
double upperFrequency = 3520.0; // A7
double gain = 15.0;

int gPixel[16] = {0,17,34,51,68,85,102,119,136,153,170,187,204,221,238,255};
int rPixel[16] = {255,238,221,204,187,170,153,136,119,102,85,68,51,34,17,0};
int bPixel[16] = {0,1,1,2,3,4,4,5,6,7,7,8,9,10,10,11};

long long current_timestamp() {
    struct timeval te; 
    gettimeofday(&te, NULL); // get current time
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // calculate milliseconds
    // printf("milliseconds: %lld\n", milliseconds);
    return milliseconds;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void onSigInt()
{
    // reset SIGINT.
    sigaction(SIGINT, &old_sigint, NULL);

    // tell main loop to exit.
    run = false;
}

// hanning window.
double windowFunction(int n, int N)
{
    return 0.5 * (1.0 - cosf(2.0 * M_PI * n / (N - 1.0)));
}

void printUsage()
{
    printf("PulseAudio PixelVloed Spectrum Analyzer\n");
    printf("\nUsage:\n");
    printf("pixelfft [options]\n");
    printf("\nOptions:\n");
    printf("-r <n>\tframes per second (default 30)\n");
    printf("-f <n>\tmaximum frequency (default 3520)\n");
    printf("-g <n>\tgain (i.e. a scaling factor for the bars, default 1.0)\n");
}

void calculateBars(fftw_complex* fft, int fftSize, int* bars, int numBars)
{
    // todo: use the float-point value and implement proper interpolation.
    double barWidthD = upperFrequency / (framesPerSecond * numBars);
    int barWidth = (int)ceil(barWidthD);

    double scale = 2.0 / fftSize * gain;

    // interpolate bars.
    int i = 0;
    for(int bar = 0; bar < numBars; bar++)
    {
        // get average.
        double power = 0.0;
        for(int j = 0; j < barWidth && i < fftSize; i++, j++)
        {
            double re = fft[i][0] * scale;
            double im = fft[i][1] * scale;
            power += re * re + im * im; // abs(c)
        }
        power *= (1.0 / barWidth); // average.
        if(power < 1e-15) power = 1e-15; // prevent overflows.

        // compute decibels.
        int dB = LINES + (int)(10.0 * log10(power));
        if(dB > LINES) dB = LINES;
        if(dB < 0) dB = 0;

        // set bar.
        bars[bar] = dB;
    }
}

int main(int argc, char* argv[])
{
    static const pa_sample_spec ss =
    {
        .format = PA_SAMPLE_FLOAT32LE,
        .rate = 44100,
        .channels = 2
    };

    // parse command line arguments.
    int c;
    while ((c = getopt(argc, argv, "r:f:g:")) != -1)
    {
        switch(c)
        {
            case 'r':
                framesPerSecond = atoi(optarg);
                break;

            case 'f':
                upperFrequency = atof(optarg);
                break;

            case 'g':
                gain = atof(optarg);
                break;

            case '?':
                printUsage();
                return 1;

            default:
                abort();
        }
    }

    //Do UDP thingies fixen
    int clientSocket, portNum;
    struct sockaddr_in serverAddr;
    socklen_t addr_size;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(5004);
    serverAddr.sin_addr.s_addr = inet_addr("10.208.42.159");
    memset(serverAddr.sin_zero, '\0', sizeof serverAddr.sin_zero);
    addr_size = sizeof serverAddr;	
    clientSocket = socket(PF_INET, SOCK_DGRAM, 0);

    // open record device
    int error;
    pa_simple *s = pa_simple_new(NULL, "pasa", PA_STREAM_RECORD, NULL, "record", &ss, NULL, NULL, &error);

    // check error
    if (!s)
    {
        fprintf(stderr, "pa_simple_new() failed: %s\n", pa_strerror(error));
        return 1;
    }

    // input buffer.
    const int size = ss.rate / framesPerSecond;
    float window[size];
    float buffer[ss.channels * size];

    // compute window.
    for(int n = 0; n < size; n++)
        window[n] = windowFunction(n, size);

    // replace SIGINT handler.
    struct sigaction sigIntAction;
    memset(&sigIntAction, 0, sizeof(sigIntAction));
    sigIntAction.sa_handler = &onSigInt;
    sigaction(SIGINT, &sigIntAction, &old_sigint);

    // fftw setup
    double *in = (double*)fftw_malloc(sizeof(double) * size);
    fftw_complex *out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * size);
    fftw_plan plan = fftw_plan_dft_r2c_1d(size, in, out, FFTW_MEASURE);

    run = true;

    int barHeight;
    struct Packet myPacket;
    myPacket.header = 0;

    sd_notify(0, "READY=1");

    // record loop
    while(run)
    {
        int barsL[COLS / 2];
        int barsR[COLS / 2];

        // read from device.
        if (pa_simple_read(s, buffer, sizeof(buffer), &error) < 0)
        {
            pa_simple_free(s);
            fprintf(stderr, "pa_simple_read() failed: %s\n", pa_strerror(error));
            return 1;
        }

        // left input.
        for (int i = 0; i < size; i++) {
            in[i] = (double)(window[i] * buffer[i * 2]);
        }
        fftw_execute(plan);
        calculateBars(out, size, barsL, COLS / 2);

        // right input.
        for (int i = 0; i < size; i++) in[i] = (double)(window[i] * buffer[i * 2 + 1]);
        fftw_execute(plan);
        calculateBars(out, size, barsR, COLS / 2);

        // draw left
        for(int i = 0; i < COLS / 2; i++)
        {
            barHeight = (int)map((int)barsL[i], 0, LINES, 15, 0);
            if (barHeight <= peakHold[i]) {
                peakHold[i] = barHeight;
                peakHoldTime[i] = current_timestamp();
            }
            if ((current_timestamp() - peakHoldTime[i]) > peakHoldTimeout) {
                peakHold[i] = barHeight;
            }
            for(int x = 0; x < 16; x++){
                myPacket.pixel[x].y = (uint16_t)(COLS / 2) - i;
                myPacket.pixel[x].x = (uint16_t)x + 16;
                if (x >= barHeight) {
                    myPacket.pixel[x].r = rPixel[x];
                    myPacket.pixel[x].g = gPixel[x];
                    myPacket.pixel[x].b = bPixel[x];
                }
                else
                {
                    myPacket.pixel[x].r = 0;
                    myPacket.pixel[x].g = 0;
                    myPacket.pixel[x].b = 0; 
                }
                if (x == peakHold[i]) {
                    myPacket.pixel[x].r = 255;
                    myPacket.pixel[x].g = 0;
                    myPacket.pixel[x].b = 0;
                }
            }
            sendto(clientSocket,(void*)&myPacket,sizeof(myPacket),0,(struct sockaddr *)&serverAddr,addr_size);
        }

        // draw right.
        for(int i = 0; i < COLS / 2; i++)
        {
            barHeight = (int)map((int)barsR[i], 0, LINES, 16, 0);
            if (barHeight <= peakHold[i]) {
                peakHold[i] = barHeight;
                peakHoldTime[i] = current_timestamp();
            }
            if ((current_timestamp() - peakHoldTime[i]) > peakHoldTimeout) {
                peakHold[i] = barHeight;
            }
            for(int x = 0; x < 16; x++){
                myPacket.pixel[x].y = (uint16_t)(COLS / 2) + i;
                myPacket.pixel[x].x = (uint16_t)x + 16;
                if (x >= barHeight) {
                    myPacket.pixel[x].r = rPixel[x];
                    myPacket.pixel[x].g = gPixel[x];
                    myPacket.pixel[x].b = bPixel[x];
                }
                else
                {
                    myPacket.pixel[x].r = 0;
                    myPacket.pixel[x].g = 0;
                    myPacket.pixel[x].b = 0; 
                }
                if (x == peakHold[i]) {
                    myPacket.pixel[x].r = 255;
                    myPacket.pixel[x].g = 0;
                    myPacket.pixel[x].b = 0;
                }
            }
            sendto(clientSocket,(void*)&myPacket,sizeof(myPacket),0,(struct sockaddr *)&serverAddr,addr_size);
        }
    }

    // clean up fftw
    fftw_destroy_plan(plan);
    fftw_free(in);
    fftw_free(out);

    // clean up pulseaudio
    pa_simple_free(s);

    return 0;
}


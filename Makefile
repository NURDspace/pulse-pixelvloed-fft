all:
		gcc pixelfft.c -o pixelfft -lm -lpulse -lpulse-simple -lfftw3

clean:
		rm pixelfft 


#ifndef WRITERFILE_H
#define WRITERFILE_H

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <stdarg.h>
#include <iostream>

#ifndef FALSE
#define FALSE 0
#endif /* !FALSE */

#ifndef TRUE
#define TRUE 1
#endif /* !TRUE */

/*----------------------------------------------------------------------------*/
/* memory allocation, print an error and exit if fail
*/
static void* xmalloc(size_t size)
{
	void* p;
	if (size == 0) {
		std::cerr << "xmalloc input: zero size" << std::endl;
	}
	p = malloc(size);
	if (p == NULL) {
		std::cerr << "out of memory" << std::endl;
	}
	return p;
}

/* open file, print an error and exit if fail
*/
static FILE* xfopen(const char* path, const char* mode)
{
	FILE* f = fopen(path, mode);
	if (f == NULL)
	{
		fprintf(stderr, "error: unable to open file '%s'\n", path);
		exit(EXIT_FAILURE);
	}
	return f;
}

/* close file, print an error and exit if fail
*/
static int xfclose(FILE* f)
{
	if (fclose(f) == EOF) {
		std::cerr << "unable to close file" << std::endl;
	}
	return 0;
}

/* skip white characters and comments in a PGM file
*/
static void skip_whites_and_comments(FILE* f)
{
	int c;
	do
	{
		while (isspace(c = getc(f))); /* skip spaces */
		if (c == '#') /* skip comments */
			while (c != '\n' && c != '\r' && c != EOF)
				c = getc(f);
	} while (c == '#' || isspace(c));
	if (c != EOF && ungetc(c, f) == EOF) {
		std::cerr << "unable to 'ungetc' while reading PGM file." << std::endl;
	}

}

/* read a number in ASCII from a PGM file
*/
static int get_num(FILE* f)
{
	int num, c;

	while (isspace(c = getc(f)));
	if (!isdigit(c)) {
		std::cerr << "corrupted PGM or PPM file." << std::endl;
	}
	num = c - '0';
	while (isdigit(c = getc(f))) num = 10 * num + c - '0';
	if (c != EOF && ungetc(c, f) == EOF) {
		std::cerr << "unable to 'ungetc' while reading PGM file." << std::endl;
	}


	return num;
}

/* read a PGM image file
*/
double * read_pgm_image(char * name, int * X, int * Y);

/*----------------------------------------------------------------------------*/
/* read a 2D ASC format file
*/
double * read_asc_file(char * name, int * X, int * Y);

/*----------------------------------------------------------------------------*/
/* read an image from a file in ASC or PGM formats
*/
double * read_image(char * name, int * X, int * Y);
/*----------------------------------------------------------------------------*/
/* write curves into a SVG file
*/
void write_curves_svg(double * x, double * y, int * curve_limits, int M,
	const char * filename, int X, int Y, double width,int x_point, int y_point, int Width, int Height);
#endif
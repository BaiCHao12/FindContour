#include "../include/WriteFile.h"

/* read a PGM image file
*/
double * read_pgm_image(char * name, int * X, int * Y)
{
	FILE * f;
	int i, n, depth, bin = FALSE;
	double * image;

	/* open file */
	f = xfopen(name, "rb"); /* open to read as a binary file (b option). otherwise,
							in some systems, it may behave differently */

	/* read header */
	if (getc(f) != 'P') {
		std::cerr <<"not a PGM file!"<<std::endl;
	}
	if ((n = getc(f)) == '2') bin = FALSE;
	else if (n == '5') bin = TRUE;
	else {
		std::cerr << "not a PGM file!" << std::endl;
	}
	skip_whites_and_comments(f);
	*X = get_num(f);               /* X size */
	skip_whites_and_comments(f);
	*Y = get_num(f);               /* Y size */
	skip_whites_and_comments(f);
	depth = get_num(f);            /* pixel depth */
	if (depth < 0) {
		std::cerr << "pixel depth < 0, unrecognized PGM file" << std::endl;
	}
	if (bin && depth > 255) {
		std::cerr << "pixel depth > 255, unrecognized PGM file" << std::endl;;
	}
	/* white before data */
	if (!isspace(getc(f))) {
		std::cerr << "corrupted PGM file." << std::endl;;
	}

	/* get memory */
	image = (double *)xmalloc(*X * *Y * sizeof(double));

	/* read data */
	for (i = 0; i<(*X * *Y); i++)
		image[i] = (double)(bin ? getc(f) : get_num(f));

	/* close file */
	xfclose(f);

	/* return image */
	return image;
}

/*----------------------------------------------------------------------------*/
/* read a 2D ASC format file
*/
double * read_asc_file(char * name, int * X, int * Y)
{
	FILE * f;
	int i, n, Z, C;
	double val;
	double * image;

	/* open file */
	f = xfopen(name, "rb"); /* open to read as a binary file (b option). otherwise,
							in some systems, it may behave differently */

	/* read header */
	n = fscanf(f, "%d%*c%d%*c%d%*c%d", X, Y, &Z, &C);
	if (n != 4 || *X <= 0 || *Y <= 0 || Z <= 0 || C <= 0) {
		std::cerr << "invalid ASC file"<<std::endl;
	}

	/* only gray level images are handled */
	if (Z != 1 || C != 1) {
		std::cerr <<"only single channel ASC files are handled"<<std::endl;
	}

	/* get memory */
	image = (double *)xmalloc(*X * *Y * Z * C * sizeof(double));

	/* read data */
	for (i = 0; i<(*X * *Y * Z * C); i++)
	{
		n = fscanf(f, "%lf", &val);
		if (n != 1) {
			std::cerr << "invalid ASC file" << std::endl;
		}
		image[i] = val;
	}

	/* close file */
	xfclose(f);

	return image;
}

/*----------------------------------------------------------------------------*/
/* read an image from a file in ASC or PGM formats
*/
double * read_image(char * name, int * X, int * Y)
{
	int n = (int)strlen(name);
	char * ext = name + n - 4;

	if (n >= 4 && (strcmp(ext, ".asc") == 0 || strcmp(ext, ".ASC") == 0))
		return read_asc_file(name, X, Y);

	return read_pgm_image(name, X, Y);
}

/*----------------------------------------------------------------------------*/
/* write curves into a SVG file
*/
void write_curves_svg(double * x, double * y, int * curve_limits, int M,
	const char * filename, int X, int Y, double width,int x_point, int y_point, int Width, int Height)
{
	FILE * svg;
	int i, k;

	/* check input */
	if (filename == NULL) {
		std::cerr<<"invalid filename in write_curves_svg" << std::endl;
	}
	if (M > 0 && (x == NULL || y == NULL || curve_limits == NULL)) {
		std::cerr << "invalid curves data in write_curves_svg"<<std::endl;;
	}
		
	if (X <= 0 || Y <= 0) {
		std::cerr << "invalid image size in write_curves_svg" << std::endl;
	}
	

	/* open file */
	svg = xfopen(filename, "wb"); /* open to write as a binary file (b option).
								  otherwise, in some systems,
								  it may behave differently */

	/* write SVG header */
	fprintf(svg, "<?xml version=\"1.0\" standalone=\"no\"?>\n");
	fprintf(svg, "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\"\n");
	fprintf(svg, " \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n");
	fprintf(svg, "<svg width=\"%dpx\" height=\"%dpx\" ", Width, Height);
	fprintf(svg, "version=\"1.1\"\n xmlns=\"http://www.w3.org/2000/svg\" ");
	fprintf(svg, "xmlns:xlink=\"http://www.w3.org/1999/xlink\">\n");


	///* write curves */
	//for (k = 0; k<M; k++) /* write curves */
	//{
	//	fprintf(svg, "<polyline stroke-width=\"%g\" ", width);
	//	fprintf(svg, "fill=\"none\" stroke=\"black\" points=\"");
	//	for (i = curve_limits[k]; i<curve_limits[k + 1]; i++)
	//		fprintf(svg, "%g,%g ", x[i], y[i]);
	//	fprintf(svg, "\"/>\n"); /* end of chain */
	//}
	fprintf(svg, "<path stroke-width=\"%g\" fill=\"none\" stroke=\"black\" d=\"", width);
	for (k = 0; k < M; k++) {
		for (i = curve_limits[k]; i < curve_limits[k + 1]; i++) {
			if (i == curve_limits[k]) {
				fprintf(svg, "M %g %g ", x[i]+x_point, y[i]+y_point); // Move to the start of the curve
			}
			else {
				fprintf(svg, "L %g %g ", x[i]+x_point, y[i]+y_point); // Draw line to the next point
			}
		}
	}
	fprintf(svg, "\"/>\n"); /* end of path */
	/* close SVG file */
	fprintf(svg, "</svg>\n");
	xfclose(svg);
}
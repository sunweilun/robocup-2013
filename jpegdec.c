/*
 * jpegdec.c
 *
 * edited from example.c in libjpeg
 *
 * This file illustrates how to use the IJG code as a subroutine library
 * to read or write JPEG image files.  You should look at this code in
 * conjunction with the documentation file libjpeg.doc.
 *
 * This code will not do anything useful as-is, but it may be helpful as a
 * skeleton for constructing routines that call the JPEG library.
 *
 * We present these routines in the same coding style used in the JPEG code
 * (ANSI function definitions, etc); but you are of course free to code your
 * routines in a different style if you prefer.
 */

#define DEBUG 0

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/*
 * Include file for users of JPEG library.
 * You will need to have included system headers that define at least
 * the typedefs FILE and size_t before you can include jpeglib.h.
 * (stdio.h is sufficient on ANSI-conforming systems.)
 * You may also wish to include "jerror.h".
 */

#include "jpeglib.h"

/*
 * <setjmp.h> is used for the optional error recovery mechanism shown in
 * the second part of the example.
 */

#include <setjmp.h>

#include "motor/include/jpegdec.h"
//#include "libmjpeg.h"
#include "motor/include/utils.h"


int fatal_error=0;

//////////////////////////dvd@9# ADD FILES BEGIN/////////////////////
////////////////////////// FROM LIBQUICKTIME/plugins/mjpeg////////////////////


/**********************************************************
 * Default huffman table generation: Ported from mjpegtools with
 * permission of the original author
 **********************************************************/

static void add_huff_table (j_decompress_ptr dinfo,
                            JHUFF_TBL **htblptr,
                            const UINT8 *bits, const UINT8 *val)
/* Define a Huffman table */
{
    int nsymbols, len;

    if (*htblptr == NULL)
        *htblptr = jpeg_alloc_huff_table((j_common_ptr) dinfo);

    /* Copy the number-of-symbols-of-each-code-length counts */
    memcpy((*htblptr)->bits, bits, sizeof((*htblptr)->bits));

    /* Validate the counts.  We do this here mainly so we can copy the right
     * number of symbols from the val[] array, without risking marching off
     * the end of memory.  jchuff.c will do a more thorough test later.
     */
    nsymbols = 0;
    for (len = 1; len <= 16; len++)
        nsymbols += bits[len];
    if (nsymbols < 1 || nsymbols > 256)
        //    mjpeg_error_exit1("jpegutils.c:  add_huff_table failed badly. ");
        //lqt_log(NULL, LQT_LOG_ERROR, LOG_DOMAIN, "add_huff_table failed badly.\n");
        if (DEBUG)fprintf(stderr,"add_huff_table failed badly.\n");
    memcpy((*htblptr)->huffval, val, nsymbols * sizeof(UINT8));
}


static void std_huff_tables (j_decompress_ptr dinfo)
/* Set up the standard Huffman tables (cf. JPEG standard section K.3) */
/* IMPORTANT: these are only valid for 8-bit data precision! */
{
    static const UINT8 bits_dc_luminance[17] = { /* 0-base */ 0, 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0 };
    static const UINT8 val_dc_luminance[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

    static const UINT8 bits_dc_chrominance[17] = { /* 0-base */ 0, 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0 };
    static const UINT8 val_dc_chrominance[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

    static const UINT8 bits_ac_luminance[17] = { /* 0-base */ 0, 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d };
    static const UINT8 val_ac_luminance[] = { 0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
                                            0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
                                            0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
                                            0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
                                            0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
                                            0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
                                            0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
                                            0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
                                            0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
                                            0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
                                            0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
                                            0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
                                            0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
                                            0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
                                            0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
                                            0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
                                            0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
                                            0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
                                            0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
                                            0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
                                            0xf9, 0xfa
                                            };
    static const UINT8 bits_ac_chrominance[17] = { /* 0-base */ 0, 0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77 };
    static const UINT8 val_ac_chrominance[] = { 0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
            0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
            0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
            0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
            0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34,
            0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
            0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
            0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
            0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
            0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
            0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
            0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
            0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
            0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
            0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
            0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
            0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2,
            0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
            0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9,
            0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
            0xf9, 0xfa
                                              };

    add_huff_table(dinfo, &dinfo->dc_huff_tbl_ptrs[0],
                   bits_dc_luminance, val_dc_luminance);
    add_huff_table(dinfo, &dinfo->ac_huff_tbl_ptrs[0],
                   bits_ac_luminance, val_ac_luminance);
    add_huff_table(dinfo, &dinfo->dc_huff_tbl_ptrs[1],
                   bits_dc_chrominance, val_dc_chrominance);
    add_huff_table(dinfo, &dinfo->ac_huff_tbl_ptrs[1],
                   bits_ac_chrominance, val_ac_chrominance);
}

static void guarantee_huff_tables(j_decompress_ptr dinfo)
{
    if ( (dinfo->dc_huff_tbl_ptrs[0] == NULL) &&
            (dinfo->dc_huff_tbl_ptrs[1] == NULL) &&
            (dinfo->ac_huff_tbl_ptrs[0] == NULL) &&
            (dinfo->ac_huff_tbl_ptrs[1] == NULL) ) {
        //    mjpeg_debug( "Generating standard Huffman tables for this frame.");
        std_huff_tables(dinfo);
    }
}

//jpeg_buffer_src related
//
typedef struct {
    struct jpeg_source_mgr pub;	/* public fields */

    JOCTET * buffer;		/* start of buffer */
    int bytes;             /* total size of buffer */
} mjpeg_source_mgr;

typedef mjpeg_source_mgr* mjpeg_src_ptr;

METHODDEF(void) init_source(j_decompress_ptr cinfo)
{
}

#define   M_EOI     0xd9

METHODDEF(boolean) fill_input_buffer(j_decompress_ptr cinfo)
{
    mjpeg_src_ptr src = (mjpeg_src_ptr) cinfo->src;

    src->buffer[0] = (JOCTET)0xFF;
    src->buffer[1] = (JOCTET)M_EOI;
    src->pub.next_input_byte = src->buffer;
    src->pub.bytes_in_buffer = 2;


    return TRUE;
}


METHODDEF(void) skip_input_data(j_decompress_ptr cinfo, long num_bytes)
{
    mjpeg_src_ptr src = (mjpeg_src_ptr)cinfo->src;

    src->pub.next_input_byte += (size_t)num_bytes;
    src->pub.bytes_in_buffer -= (size_t)num_bytes;
}


METHODDEF(void) term_source(j_decompress_ptr cinfo)
{
}




GLOBAL(void) jpeg_buffer_src(j_decompress_ptr cinfo, unsigned char *buffer, long bytes)
{
    mjpeg_src_ptr src;

    /* first time for this JPEG object? */
    if (cinfo->src == NULL) {
        //cinfo->src = (struct jpeg_source_mgr*)
        //  (*cinfo->mem->alloc_small)((j_common_ptr)cinfo,
        //                             JPOOL_PERMANENT,
        //                             sizeof(mjpeg_source_mgr));
        cinfo->src=malloc(sizeof(mjpeg_source_mgr));
        src = (mjpeg_src_ptr)cinfo->src;
    }

    src = (mjpeg_src_ptr)cinfo->src;
    src->pub.init_source = init_source;
    src->pub.fill_input_buffer = fill_input_buffer;
    src->pub.skip_input_data = skip_input_data;
    src->pub.resync_to_restart = jpeg_resync_to_restart; /* use default method */
    src->pub.term_source = term_source;
    src->pub.bytes_in_buffer = bytes;
    src->pub.next_input_byte = buffer;
    src->buffer = buffer;
    src->bytes = bytes;
}





//
/////////////////////////dvd@9# FUNCS END





/******************** JPEG COMPRESSION SAMPLE INTERFACE *******************/

/* This half of the example shows how to feed data into the JPEG compressor.
 * We present a minimal version that does not worry about refinements such
 * as error recovery (the JPEG code will just exit() if it gets an error).
 */


/*
 * IMAGE DATA FORMATS:
 *
 * The standard input image format is a rectangular array of pixels, with
 * each pixel having the same number of "component" values (color channels).
 * Each pixel row is an array of JSAMPLEs (which typically are unsigned chars).
 * If you are working with color data, then the color values for each pixel
 * must be adjacent in the row; for example, R,G,B,R,G,B,R,G,B,... for 24-bit
 * RGB color.
 *
 * For this example, we'll assume that this data structure matches the way
 * our application has stored the image in memory, so we can just pass a
 * pointer to our image buffer.  In particular, let's say that the image is
 * RGB color and is described by:
 */

//extern JSAMPLE * image_buffer;	/* Points to large array of R,G,B-order data */
//extern int image_height;	/* Number of rows in image */
//extern int image_width;		/* Number of columns in image */


/*
 * Sample routine for JPEG compression.  We assume that the target file name
 * and a compression quality factor are passed in.
 */
//NOTICE: deleted. see example.c
//void write_JPEG_file (char * filename, int quality)



/******************** JPEG DECOMPRESSION SAMPLE INTERFACE *******************/

/* This half of the example shows how to read data from the JPEG decompressor.
 * It's a bit more refined than the above, in that we show:
 *   (a) how to modify the JPEG library's standard error-reporting behavior;
 *   (b) how to allocate workspace using the library's memory manager.
 *
 * Just to make this example a little different from the first one, we'll
 * assume that we do not intend to put the whole image into an in-memory
 * buffer, but to send it line-by-line someplace else.  We need a one-
 * scanline-high JSAMPLE array as a work buffer, and we will let the JPEG
 * memory manager allocate it for us.  This approach is actually quite useful
 * because we don't need to remember to deallocate the buffer separately: it
 * will go away automatically when the JPEG object is cleaned up.
 */


/*
 * ERROR HANDLING:
 *
 * The JPEG library's standard error handler (jerror.c) is divided into
 * several "methods" which you can override individually.  This lets you
 * adjust the behavior without duplicating a lot of code, which you might
 * have to update with each future release.
 *
 * Our example here shows how to override the "error_exit" method so that
 * control is returned to the library's caller when a fatal error occurs,
 * rather than calling exit() as the standard error_exit method does.
 *
 * We use C's setjmp/longjmp facility to return control.  This means that the
 * routine which calls the JPEG library must first execute a setjmp() call to
 * establish the return point.  We want the replacement error_exit to do a
 * longjmp().  But we need to make the setjmp buffer accessible to the
 * error_exit routine.  To do this, we make a private extension of the
 * standard JPEG error handler object.  (If we were using C++, we'd say we
 * were making a subclass of the regular error handler.)
 *
 * Here's the extended error handler struct:
 */

struct my_error_mgr {
    struct jpeg_error_mgr pub;	/* "public" fields */

    jmp_buf setjmp_buffer;	/* for return to caller */
};

typedef struct my_error_mgr * my_error_ptr;

/*
 * Here's the routine that will replace the standard error_exit method:
 */

METHODDEF(void)
my_error_exit (j_common_ptr cinfo)
{
    if (DEBUG)printf("my_error_exit()\n");
    /* cinfo->err really points to a my_error_mgr struct, so coerce pointer */
    my_error_ptr myerr = (my_error_ptr) cinfo->err;

    /* Always display the message. */
    /* We could postpone this until after returning, if we chose. */
    (*cinfo->err->output_message) (cinfo);
    if (DEBUG)printf("myerr->setjmp_buffer\n");
    /* Return control to the setjmp point */
    fatal_error=1;
    return;
    //longjmp(myerr->setjmp_buffer, 1);
    //if(DEBUG)printf("my_error_exit_exit\n");
}



JSAMPARRAY buffer;		/* Output row buffer */



//Return 1 if an fatal error occurs;
//If it is true, decoding process can not continue any more;
//Aplication should destroy jpeg_decompress_struct function,
//and re-build the whole jpeg_decompress_struct.
int isfatalerror()
{
    return fatal_error;
}

struct my_error_mgr *jerr;

void deleteDecompressor(struct jpeg_decompress_struct* cinfo)
{
    /* Step 8: Release JPEG decompression object */

    /* This is an important step since it will release a good deal of memory. */
    jpeg_destroy_decompress(cinfo);

    /* After finish_decompress, we can close the input file.
     * Here we postpone it until after no more JPEG errors are possible,
     * so as to simplify the setjmp error logic above.  (Actually, I don't
     * think that jpeg_destroy can do an error exit, but why assume anything...)
     */
    //fclose(infile);

    /* At this point you may want to check to see whether any corrupt-data
     * warnings occurred (test whether jerr.pub.num_warnings is nonzero).
     */

    free(cinfo);
    free(buffer[0]);
    free(buffer);
    free(jerr);
    //free(buffer);

}
struct jpeg_decompress_struct* newDecompressor(int width) {

    /* This struct contains the JPEG decompression parameters and pointers to
     * working space (which is allocated as needed by the JPEG library).
     */
    struct jpeg_decompress_struct *cinfo=malloc(sizeof(struct jpeg_decompress_struct));
    /* We use our private extension JPEG error handler.
     * Note that this struct must live as long as the main JPEG parameter
     * struct, to avoid dangling-pointer problems.
     */
    jerr=malloc(sizeof(struct my_error_mgr));
    /* More stuff */
    //FILE * infile;		/* source file */
    //JSAMPARRAY buffer;		/* Output row buffer */
    //int row_stride;		/* physical row width in output buffer */

    /* In this example we want to open the input file before doing anything else,
     * so that the setjmp() error recovery below can assume the file is open.
     * VERY IMPORTANT: use "b" option to fopen() if you are on a machine that
     * requires it in order to read binary files.
     */
    /*
    if ((infile = fopen(filename, "rb")) == NULL) {
      if(DEBUG)fprintf(stderr, "can't open %s\n", filename);
      return 0;
    }
    */

    /* Step 1: allocate and initialize JPEG decompression object */

    /* We set up the normal JPEG error routines, then override error_exit. */
    cinfo->err = jpeg_std_error(&jerr->pub);
    jerr->pub.error_exit = my_error_exit;
    /* Establish the setjmp return context for my_error_exit to use. */
    if (setjmp(jerr->setjmp_buffer)) {
        /* If we get here, the JPEG code has signaled an error.
         * We need to clean up the JPEG object, close the input file, and return.
         */
        jpeg_destroy_decompress(cinfo);
        //fclose(infile);
        return 0;
    }
    /* Now we can initialize the JPEG decompression object. */
    jpeg_create_decompress(cinfo);

    //make buffer
    /* Make a one-row-high sample array that will go away when done with image */
    //buffer = (*cinfo->mem->alloc_sarray)
// 		((j_common_ptr) cinfo, JPOOL_IMAGE, width, 1);
    buffer=malloc(sizeof(unsigned char*));
    buffer[0]=malloc(width*3);

    //set fatal error marker to zero
    fatal_error=0;
    return cinfo;
}

/*
 * Sample routine for JPEG decompression.  We assume that the source file name
 * is passed in.  We want to return 1 on success, 0 on error.
 */


int read_JPEG_buffer_from_file(struct jpeg_decompress_struct *cinfo, //header
                               //unsigned char*jpgbuffer, //src buffer, stores mjpeg
                               //	long buffersize, //src buffer size
                               FILE* file,
                               unsigned char* destbuffer, //destination buffer, rgb
                               long destbuffersize,//buffer size
                               struct img_info* image_info)//img info
{
    int row_stride;		/* physical row width in output buffer */



    /* Step 2: specify data source (eg, a file) */

    if (DEBUG)printf("----------------jpeg_file_src--------------\n");
    //jpeg_buffer_src(cinfo, jpgbuffer,buffersize);
    jpeg_stdio_src(cinfo,file);

    if (DEBUG)printf("----------------after jpeg_file_src--------------\n");

    /* Step 3: read file parameters with jpeg_read_header() */

    int headerret= jpeg_read_header(cinfo, TRUE);
    if (DEBUG)printf("jpeg_read_header %d %d\n",headerret,JPEG_HEADER_OK);
    /* We can ignore the return value from jpeg_read_header since
     *   (a) suspension is not possible with the stdio data source, and
     *   (b) we passed TRUE to reject a tables-only JPEG file as an error.
     * See libjpeg.doc for more info.
     */

    //debug info: output header information
    /*
    if(DEBUG)printf("JPEG PARAMETERS:\n");
    if(DEBUG)printf("Width Height %d %d\n",cinfo->image_width,
      cinfo->image_height);
    if(DEBUG)printf("COLOR SPACE %d %d\n",cinfo->jpeg_color_space,JCS_YCbCr);

    if(DEBUG)printf("OUT COLOR SPCAE: %d\n",cinfo->out_color_space);

    if(DEBUG)printf("QUANTIZE COLORS: %d\n",cinfo->quantize_colors);
    */
    if (image_info!=NULL) {
        image_info->width=cinfo->output_width;
        image_info->height=cinfo->output_height;
    }
    if (DEBUG)printf("fatal error %d\n",fatal_error);

    //for mjpeg pictures, huffman table must be added.
    //extracted from libquicktime
    if (DEBUG)printf("guarantee_huff_tables\n");
    guarantee_huff_tables(cinfo);


    /* Step 4: set parameters for decompression */

    /* In this example, we don't need to change any of the defaults set by
     * jpeg_read_header(), so we do nothing here.
     */

    /* Step 5: Start decompressor */

    if (DEBUG)printf("(void) jpeg_start_decompress(cinfo);\n");
    (void) jpeg_start_decompress(cinfo);
    /* We can ignore the return value since suspension is not possible
     * with the stdio data source.
     */

    /* We may need to do some setup of our own at this point before reading
     * the data.  After jpeg_start_decompress() we have the correct scaled
     * output image dimensions available, as well as the output colormap
     * if we asked for color quantization.
     * In this example, we need to make an output work buffer of the right size.
     */
    /* JSAMPLEs per row in output buffer */
    row_stride = cinfo->output_width * cinfo->output_components;

    //debug info: output_components must be 3, yuv or rgb
    //	if(DEBUG)printf("row_stride: %d output component:%d \n",
    //		row_stride,cinfo->output_components);
    /* Make a one-row-high sample array that will go away when done with image */
    //buffer = (*cinfo->mem->alloc_sarray)
    //		((j_common_ptr) cinfo, JPOOL_IMAGE, row_stride, 1);


    //output rgb buffer
    long offset=0;
    //long offset=200;
    //unsigned char* filebuf=malloc(row_stride*cinfo->output_height+offset);
    //if(DEBUG)sprintf(filebuf,"%d %d %d\ntype rgb, offset 0, width 1, height 2\n",offset,cinfo->output_width,cinfo->output_height);
    long start=0;
    /* Step 6: while (scan lines remain to be read) */
    /*           jpeg_read_scanlines(...); */

    /* Here we use the library's state variable cinfo.output_scanline as the
     * loop counter, so that we don't have to keep track ourselves.
     */
    int read_line=0;
    if (DEBUG)printf("loop while\n");
    while (cinfo->output_scanline < cinfo->output_height) {
        /* jpeg_read_scanlines expects an array of pointers to scanlines.
         * Here the array is only one element long, but you could ask for
         * more than one scanline at a time if that's more convenient.
         */
        if (DEBUG)printf("read_scan_lines...%d\n",cinfo->output_scanline);
        read_line=jpeg_read_scanlines(cinfo, buffer,1);// cinfo.image_height);

        //debug info
        if (DEBUG)printf("output_scan line %d height %d read lines: %d\n",
                             cinfo->output_scanline,
                             cinfo->output_height,read_line);

        /* Assume put_scanline_someplace wants a pointer and sample count. */
        //put_scanline_someplace(buffer[0], row_stride);
        memcpy(destbuffer+offset+start,buffer[0],row_stride);
        start+=row_stride;
        if (start>destbuffersize) {
            if (DEBUG)fprintf(stderr,"destination buffer overflow while decompressing jpeg files.\n");
            exit(-1);
        }
    }

    //debug
    //msave("1.tmp",filebuf,start);
    /* Step 7: Finish decompression */
    if (DEBUG)printf("(void) jpeg_finish_decompress(cinfo)\n");

    (void) jpeg_finish_decompress(cinfo);
    /* We can ignore the return value since suspension is not possible
     * with the stdio data source.
     */

    /* And we're done! */
    return 1;
}


/*
 * Sample routine for JPEG decompression.  We assume that the source file name
 * is passed in.  We want to return 1 on success, 0 on error.
 */


int read_JPEG_buffer(struct jpeg_decompress_struct *cinfo, //header
                     unsigned char*jpgbuffer, //src buffer, stores mjpeg
                     long buffersize, //src buffer size
                     unsigned char* destbuffer, //destination buffer, rgb
                     long destbuffersize,//buffer size
                     struct img_info* image_info , //image info
                     int format)
{
    int row_stride;		/* physical row width in output buffer */


    if (DEBUG)printf("come in\n");
    /* Step 2: specify data source (eg, a file) */

    if (DEBUG)printf("----------------jpeg_buffer_src--------------\n");
    jpeg_buffer_src(cinfo, jpgbuffer,buffersize);
    if (DEBUG)printf("----------------after jpeg_buffer_src--------------\n");

    /* Step 3: read file parameters with jpeg_read_header() */
    if (DEBUG)printf("read header...\n");
    int headerret= jpeg_read_header(cinfo, TRUE);
    if (DEBUG)printf("jpeg_read_header %d %d\n",headerret,JPEG_HEADER_OK);
    if (DEBUG)printf("read header end...\n");
    /* We can ignore the return value from jpeg_read_header since
     *   (a) suspension is not possible with the stdio data source, and
     *   (b) we passed TRUE to reject a tables-only JPEG file as an error.
     * See libjpeg.doc for more info.
     */

    //debug info: output header information
    /*
    if(DEBUG)printf("JPEG PARAMETERS:\n");
    if(DEBUG)printf("Width Height %d %d\n",cinfo->image_width,
      cinfo->image_height);
    if(DEBUG)printf("COLOR SPACE %d %d\n",cinfo->jpeg_color_space,JCS_YCbCr);

    if(DEBUG)printf("OUT COLOR SPCAE: %d\n",cinfo->out_color_space);

    if(DEBUG)printf("QUANTIZE COLORS: %d\n",cinfo->quantize_colors);
    */
    if (format==1)
        cinfo->out_color_space=JCS_YCbCr;
    if (image_info!=NULL) {
        image_info->width=cinfo->image_width;
        image_info->height=cinfo->image_height;
        if (DEBUG)printf("Width Height %d %d\n",cinfo->image_width,
                             cinfo->image_height);
    }

    if (fatal_error) {
        if (DEBUG)printf("fatal error %d\n",fatal_error);
        return -1;
    }
    //for mjpeg pictures, huffman table must be added.
    //extracted from libquicktime
    if (DEBUG)printf("guarantee_huff_tables\n");
    guarantee_huff_tables(cinfo);


    /* Step 4: set parameters for decompression */

    /* In this example, we don't need to change any of the defaults set by
     * jpeg_read_header(), so we do nothing here.
     */

    /* Step 5: Start decompressor */

    (void) jpeg_start_decompress(cinfo);
    /* We can ignore the return value since suspension is not possible
     * with the stdio data source.
     */

    /* We may need to do some setup of our own at this point before reading
     * the data.  After jpeg_start_decompress() we have the correct scaled
     * output image dimensions available, as well as the output colormap
     * if we asked for color quantization.
     * In this example, we need to make an output work buffer of the right size.
     */
    /* JSAMPLEs per row in output buffer */
    row_stride = cinfo->output_width * cinfo->output_components;

    //debug info: output_components must be 3, yuv or rgb
    //	if(DEBUG)printf("row_stride: %d output component:%d \n",
    //		row_stride,cinfo->output_components);
    /* Make a one-row-high sample array that will go away when done with image */
    //buffer = (*cinfo->mem->alloc_sarray)
    //		((j_common_ptr) cinfo, JPOOL_IMAGE, row_stride, 1);


    //output rgb buffer
    long offset=0;
    //long offset=200;
    //unsigned char* filebuf=malloc(row_stride*cinfo->output_height+offset);
    //sif(DEBUG)printf(filebuf,"%d %d %d\ntype rgb, offset 0, width 1, height 2\n",offset,cinfo->output_width,cinfo->output_height);
    long start=0;
    /* Step 6: while (scan lines remain to be read) */
    /*           jpeg_read_scanlines(...); */

    /* Here we use the library's state variable cinfo.output_scanline as the
     * loop counter, so that we don't have to keep track ourselves.
     */
    int read_line=0;
    while (cinfo->output_scanline < cinfo->output_height) {
        /* jpeg_read_scanlines expects an array of pointers to scanlines.
         * Here the array is only one element long, but you could ask for
         * more than one scanline at a time if that's more convenient.
         */
        read_line=jpeg_read_scanlines(cinfo, buffer,1);// cinfo.image_height);

        //debug info
        //if(DEBUG)printf("output_scan line %d height %d read lines: %d\n",
        //	cinfo->output_scanline,
        //	cinfo->output_height,read_line);

        /* Assume put_scanline_someplace wants a pointer and sample count. */
        //put_scanline_someplace(buffer[0], row_stride);
        memcpy(destbuffer+offset+start,buffer[0],row_stride);
        start+=row_stride;
        if (start>destbuffersize) {
            if (DEBUG)fprintf(stderr,"destination buffer overflow while decompressing jpeg files.\n");
            exit(-1);
        }
    }

    //debug
    //msave("1.tmp",filebuf,start);
    /* Step 7: Finish decompression */

    (void) jpeg_finish_decompress(cinfo);
    /* We can ignore the return value since suspension is not possible
     * with the stdio data source.
     */

    /* And we're done! */
    return 1;
}

/*
 * SOME FINE POINTS:
 *
 * In the above code, we ignored the return value of jpeg_read_scanlines,
 * which is the number of scanlines actually read.  We could get away with
 * this because we asked for only one line at a time and we weren't using
 * a suspending data source.  See libjpeg.doc for more info.
 *
 * We cheated a bit by calling alloc_sarray() after jpeg_start_decompress();
 * we should have done it beforehand to ensure that the space would be
 * counted against the JPEG max_memory setting.  In some systems the above
 * code would risk an out-of-memory error.  However, in general we don't
 * know the output image dimensions before jpeg_start_decompress(), unless we
 * call jpeg_calc_output_dimensions().  See libjpeg.doc for more about this.
 *
 * Scanlines are returned in the same order as they appear in the JPEG file,
 * which is standardly top-to-bottom.  If you must emit data bottom-to-top,
 * you can use one of the virtual arrays provided by the JPEG memory manager
 * to invert the data.  See wrbmp.c for an example.
 *
 * As with compression, some operating modes may require temporary files.
 * On some systems you may need to set up a signal handler to ensure that
 * temporary files are deleted if the program is interrupted.  See libjpeg.doc.
 */






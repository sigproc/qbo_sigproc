/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

#ifndef SEGMENT_H
#define SEGMENT_H

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <climits>
#include <cmath>
#include <fstream>
#include <vector>
#include <algorithm>
#include "segment.h"
#include <string>

//Headers taken from each file

/***************misc.h***********************/

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

typedef unsigned char uchar;

typedef struct { uchar r, g, b; } rgb;

inline bool operator==(const rgb &a, const rgb &b) {
  return ((a.r == b.r) && (a.g == b.g) && (a.b == b.b));
}

template <class T>
inline T abs(const T &x) { return (x > 0 ? x : -x); };

template <class T>
inline int sign(const T &x) { return (x >= 0 ? 1 : -1); };

template <class T>
inline T square(const T &x) { return x*x; };

template <class T>
inline T bound(const T &x, const T &min, const T &max) {
  return (x < min ? min : (x > max ? max : x));
}

template <class T>
inline bool check_bound(const T &x, const T&min, const T &max) {
  return ((x < min) || (x > max));
}

inline int vlib_round(float x) { return (int)(x + 0.5F); }

inline int vlib_round(double x) { return (int)(x + 0.5); }

inline double gaussian(double val, double sigma) {
  return exp(-square(val/sigma)/2)/(sqrt(2*M_PI)*sigma);
}

/***************image.h***********************/

template <class T>
class image {
 public:
  /* create an image */
  image(const int width, const int height, const bool init = true);

  /* delete an image */
  ~image();

  /* init an image */
  void init(const T &val);

  /* copy an image */
  image<T> *copy() const;
  
  /* get the width of an image. */
  int width() const { return w; }
  
  /* get the height of an image. */
  int height() const { return h; }
  
  /* image data. */
  T *data;
  
  /* row pointers. */
  T **access;
  
 private:
  int w, h;
};

/* use imRef to access image data. */
#define imRef(im, x, y) (im->access[y][x])
  
/* use imPtr to get pointer to image data. */
#define imPtr(im, x, y) &(im->access[y][x])

template <class T>
image<T>::image(const int width, const int height, const bool init) {
  w = width;
  h = height;
  data = new T[w * h];  // allocate space for image data
  access = new T*[h];   // allocate space for row pointers
  
  // initialize row pointers
  for (int i = 0; i < h; i++)
    access[i] = data + (i * w);  
  
  if (init)
    memset(data, 0, w * h * sizeof(T));
}

template <class T>
image<T>::~image() {
  delete [] data; 
  delete [] access;
}

template <class T>
void image<T>::init(const T &val) {
  T *ptr = imPtr(this, 0, 0);
  T *end = imPtr(this, w-1, h-1);
  while (ptr <= end)
    *ptr++ = val;
}


template <class T>
image<T> *image<T>::copy() const {
  image<T> *im = new image<T>(w, h, false);
  memcpy(im->data, data, w * h * sizeof(T));
  return im;
}

/***************disjoint-set.h***********************/

typedef struct {
  int rank;
  int p;
  int size;
} uni_elt;

class universe {
public:
  universe(int elements);
  ~universe();
  int find(int x);  
  void join(int x, int y);
  int size(int x) const { return elts[x].size; }
  int num_sets() const { return num; }

private:
  uni_elt *elts;
  int num;
};

/***************filter.h***********************/

#define WIDTH 4.0

/* make filters */
#define MAKE_FILTER(name, fun)                                \
static std::vector<float> make_ ## name (float sigma) {       \
  sigma = std::max(sigma, 0.01F);			      \
  int len = (int)ceil(sigma * WIDTH) + 1;                     \
  std::vector<float> mask(len);                               \
  for (int i = 0; i < len; i++) {                             \
    mask[i] = fun;                                            \
  }                                                           \
  return mask;                                                \
}

/* normalize mask so it integrates to one */
static void normalize(std::vector<float> &mask);

/* convolve image with gaussian filter */
static image<float> *smooth(image<float> *src, float sigma);

/* convolve image with gaussian filter */
image<float> *smooth(image<uchar> *src, float sigma) ;

/* compute laplacian */
static image<float> *laplacian(image<float> *src) ;




/***************imconv.h***********************/

#define	RED_WEIGHT	0.299
#define GREEN_WEIGHT	0.587
#define BLUE_WEIGHT	0.114

static image<uchar> *imageRGBtoGRAY(image<rgb> *input) ;

static image<rgb> *imageGRAYtoRGB(image<uchar> *input) ;

static image<float> *imageUCHARtoFLOAT(image<uchar> *input) ;
static image<float> *imageINTtoFLOAT(image<int> *input) ;

static image<uchar> *imageFLOATtoUCHAR(image<float> *input, float min, float max) ;

static image<uchar> *imageFLOATtoUCHAR(image<float> *input) ;

static image<long> *imageUCHARtoLONG(image<uchar> *input) ;

static image<uchar> *imageLONGtoUCHAR(image<long> *input, long min, long max) ;
static image<uchar> *imageLONGtoUCHAR(image<long> *input) ;

static image<uchar> *imageSHORTtoUCHAR(image<short> *input, short min, short max);

static image<uchar> *imageSHORTtoUCHAR(image<short> *input) ;


/***************imutil.h***********************/

/* compute minimum and maximum value in an image */
template <class T>
void min_max(image<T> *im, T *ret_min, T *ret_max) {
  int width = im->width();
  int height = im->height();
  
  T min = imRef(im, 0, 0);
  T max = imRef(im, 0, 0);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      T val = imRef(im, x, y);
      if (min > val)
	min = val;
      if (max < val)
	max = val;
    }
  }

  *ret_min = min;
  *ret_max = max;
} 

/* threshold image */
template <class T>
image<uchar> *threshold(image<T> *src, int t) {
  int width = src->width();
  int height = src->height();
  image<uchar> *dst = new image<uchar>(width, height);
  
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      imRef(dst, x, y) = (imRef(src, x, y) >= t);
    }
  }

  return dst;
}

/***************pnmfile.h***********************/

#define BUF_SIZE 256

class pnm_error { };

static void read_packed(unsigned char *data, int size, std::ifstream &f) ;
static void write_packed(unsigned char *data, int size, std::ofstream &f) ;
/* read PNM field, skipping comments */ 
static void pnm_read(std::ifstream &file, char *buf) ;

static image<uchar> *loadPBM(const char *name) ;
static void savePBM(image<uchar> *im, const char *name) ;

static image<uchar> *loadPGM(const char *name);

static void savePGM(image<uchar> *im, const char *name);

static image<rgb> *loadPPM(const char *name) ;

static void savePPM(image<rgb> *im, const char *name) ;

template <class T>
void load_image(image<T> **im, const char *name) {
  char buf[BUF_SIZE];
  
  /* read header */
  std::ifstream file(name, std::ios::in | std::ios::binary);
  pnm_read(file, buf);
  if (strncmp(buf, "VLIB", 9))
    throw pnm_error();

  pnm_read(file, buf);
  int width = atoi(buf);
  pnm_read(file, buf);
  int height = atoi(buf);

  /* read data */
  *im = new image<T>(width, height);
  file.read((char *)imPtr((*im), 0, 0), width * height * sizeof(T));
}

template <class T>
void save_image(image<T> *im, const char *name) {
  int width = im->width();
  int height = im->height();
  std::ofstream file(name, std::ios::out | std::ios::binary);

  file << "VLIB\n" << width << " " << height << "\n";
  file.write((char *)imPtr(im, 0, 0), width * height * sizeof(T));
}
/****************segment-graph.h********************/
typedef struct {
  float w;
  int a, b;
} edge;

// threshold function
#define THRESHOLD(size, c) (c/size)

universe *segment_graph(int num_vertices, int num_edges, edge *edges, float c);


/***************convolve.h***********************/
static void convolve_even(image<float> *src, image<float> *dst, std::vector<float> &mask);
static void convolve_odd(image<float> *src, image<float> *dst, std::vector<float> &mask) ;

/***************segment-image.h***********************/

// random color
rgb random_rgb();

// dissimilarity measure between pixels
static inline float diff(image<float> *r, image<float> *g, image<float> *b, int x1, int y1, int x2, int y2) ;
static inline float diff1C(image<float> *d, int x1, int y1, int x2, int y2);

image<rgb> *segment_image(image<rgb> *im, float sigma, float c, int min_size,  int *num_ccs);
/*image<rgb> *segment_image1C(image<float> *im, float sigma, float c, int min_size,
			  int *num_ccs, ) ;*/
image<rgb> *segment_image1C(image<float> *im, float sigma, float Kdepth, float Knormal, int min_size,
			  int *num_ccs, image<rgb> ** outputs0, image<rgb> ** outputs1, image<rgb> ** outputs2);

edge* create_depth_graph(image<float> *d, int *edgeNum);





//END
#endif

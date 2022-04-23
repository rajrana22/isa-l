/**********************************************************************
  Copyright(c) 2011-2015 Intel Corporation All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>		// for memset, memcmp
#include "erasure_code.h"
#include "test.h"
#include <time.h>
#define BILLION  1000000000L;

// #define CACHED_TEST
#ifdef CACHED_TEST
// Cached test, loop many times over small dataset
# define TEST_SOURCES 40
# define TEST_LEN(m)  ((128*1024 / m) & ~(64-1))
# define TEST_TYPE_STR "_warm"
#else
# ifndef TEST_CUSTOM
// Uncached test.  Pull from large mem base.
#  define TEST_SOURCES 150
#  define GT_L3_CACHE  1024*1024*1024	/* some number > last level cache */
//#  define GT_L3_CACHE  1024*1024	/* some number > last level cache */
#  define TEST_NUM(chunksize, k) ((GT_L3_CACHE / (chunksize*1024*k)))
//#  define TEST_NUM(chunksize) 1
#  define TEST_LEN(chunksize, k)  (chunksize*1024)        // k is data units #
#  define TEST_TYPE_STR "_cold"
# else
#  define TEST_TYPE_STR "_cus"
# endif
#endif

#define MMAX TEST_SOURCES
#define KMAX TEST_SOURCES

#define BAD_MATRIX -1

typedef unsigned char u8;

void ec_encode_data_stripes(int m, int m2, int k, int k2, u8 * g_tbls, u8 * g_tbls2, 
				u8 *** buffs, u8 **** parities, int len, int len2, 
				int stripes, int stripes2, double *t)
{
	int x, y;
	for (x = 0; x < stripes2; x++) {
		ec_encode_data(len2, k2, m2 - k2, g_tbls2, buffs[x], &buffs[x][k2]);
	}
}

void ec_encode_data_stripes_inner(int m, int m2, int k, int k2, u8 * g_tbls, u8 * g_tbls2, 
				u8 *** buffs, u8 **** parities, int len, int len2, 
				int stripes, int stripes2, double *t)
{
	int x, y;
	for (x = 0; x < stripes2; x++) {
		for (y = 0; y < m2; y++)
			ec_encode_data(len, k, m - k, g_tbls, parities[x][y], &parities[x][y][k]);
	}
}

void ec_encode_data_stripes_detail(int m, int k, u8 * g_tbls, u8 *** buffs, int len, int stripes)
{
	int x;
	struct timespec start, stop;
	for (x = 0; x < stripes; x++) {
		clock_gettime( CLOCK_REALTIME, &start);
		ec_encode_data(len, k, m - k, g_tbls, buffs[x], &buffs[x][k]);
		clock_gettime( CLOCK_REALTIME, &stop);
		double cost = (stop.tv_sec - start.tv_sec)+ (double)( stop.tv_nsec - start.tv_nsec )
               		/ (double)BILLION;
		printf( "%lf  x:%d stripes:%d  len:%d\n", cost, x, stripes, len);
	}
}

void ec_encode_perf(int m, int m2, int k, int k2, u8 * a, u8* a2, u8 * g_tbls, u8 * g_tbls2, 
			u8 *** buffs, u8 **** parities, struct perf *start, int len, int len2, 
			int stripes, int stripes2, double* t)
{
	printf("init ec table..\n");
	ec_init_tables(k2, m2 - k2, &a2[k2 * k2], g_tbls2);
	BENCHMARK(start, 0,
		ec_encode_data_stripes(m, m2, k, k2, g_tbls, g_tbls2, buffs, parities, len, len2, 
					stripes, stripes2, t))
}



void ec_encode_perf_inner(int m, int m2, int k, int k2, u8 * a, u8* a2, u8 * g_tbls, u8 * g_tbls2, 
			u8 *** buffs, u8 **** parities, struct perf *start, int len, int len2, 
			int stripes, int stripes2, double* t)
{
	printf("init ec table..\n");
	ec_init_tables(k, m - k, &a[k * k], g_tbls);
	BENCHMARK(start, 0,
		ec_encode_data_stripes_inner(m, m2, k, k2, g_tbls, g_tbls2, buffs, parities, len, len2, 
					stripes, stripes2, t))
}



int ec_decode_perf(int m, int k, u8 * a, u8 * g_tbls, u8 ** buffs, u8 * src_in_err,
		   u8 * src_err_list, int nerrs, u8 ** temp_buffs, struct perf *start, int chunksize)
{
	int i, j, r;
	u8 b[MMAX * KMAX], c[MMAX * KMAX], d[MMAX * KMAX];
	u8 *recov[TEST_SOURCES];

	// Construct b by removing error rows
	for (i = 0, r = 0; i < k; i++, r++) {
		while (src_in_err[r])
			r++;
		recov[i] = buffs[r];
		for (j = 0; j < k; j++)
			b[k * i + j] = a[k * r + j];
	}

	if (gf_invert_matrix(b, d, k) < 0)
		return BAD_MATRIX;

	for (i = 0; i < nerrs; i++)
		for (j = 0; j < k; j++)
			c[k * i + j] = d[k * src_err_list[i] + j];

	// Recover data
	ec_init_tables(k, nerrs, c, g_tbls);
	BENCHMARK(start, BENCHMARK_TIME,
		  ec_encode_data(TEST_LEN(chunksize,k), k, nerrs, g_tbls, recov, temp_buffs));

	return 0;
}

int main(int argc, char *argv[])
{
	int i, j, x, y, z;
	int m, k, p;
	int k2,p2,m2;
	int chunksize, stripes, stripesize, len, datasize;
	int chunksize2, stripes2, stripesize2, len2;

	// Pick test parameters
	// @meng: this means 10+4 EC
	m = 14;
	k = 10;
	p = 4;
//	const u8 err_list[] = { 2, 4, 5, 7 };
//	const u8 err_list[] = { 1, 2, 3, 4 };

	if (argc < 4)
		printf("./erasure_code_perf_erasure_code_perf data_num parity_num chunksize data_num2 parity_num2\n");

	k = atoi(argv[1]);
	p = atoi(argv[2]);
	chunksize = atoi(argv[3]);	// in kb
	k2 = atoi(argv[4]);
	p2 = atoi(argv[5]);
	chunksize2 = chunksize * k;
	len = chunksize * 1024;    // in bytes
	len2 = chunksize2 * 1024;
	stripesize = chunksize * k; // in kb
	stripesize2 = chunksize2 * k2;
	datasize = 1 * 1024 * 1024; //10 gib in kb
	stripes2 = datasize / stripesize2;
	stripes = datasize / stripesize;
	m = k + p;
	m2 = k2 + p2;
	printf("data_num:%d parity_num:%d m:%d chunksize:%dKB len:%dB stripesize:%dKB stripes:%d\n",
		k, p, m, chunksize, len, stripesize, stripes);
	printf("data_num2:%d parity_num2:%d m2:%d chunksize2:%dKB len2:%dB stripesize2:%dKB stripes2:%d\n",
		k2, p2, m2, chunksize2, len2, stripesize2, stripes2);

	printf("erasure_code_perf_erasure_code_perf: %dx%d %d\n", m, len, p);

/*	u8 *temp_buffs[TEST_SOURCES], *buffs[TEST_SOURCES];
	u8 a[MMAX * KMAX];
	u8 g_tbls[KMAX * TEST_SOURCES * 32];//, src_in_err[TEST_SOURCES];
//	u8 src_err_list[TEST_SOURCES];
*/
	struct perf start;

	printf("stripes:%d\n", stripes);
	printf("stripes2:%d\n", stripes2);

	u8*** buffs2 = (u8***) malloc(stripes2 * sizeof(u8**));
	for (x = 0; x < stripes2; x++)
		buffs2[x] = (u8**) malloc(m2 * sizeof(u8*));

	u8**** parities = (u8****) malloc(stripes2 * sizeof(u8***));
	for (x = 0; x < stripes2; x++) {
		parities[x] = (u8***) malloc(m2 * sizeof(u8**));
		for (y = 0; y < m2; y++)
			parities[x][y] = (u8**) malloc(m * sizeof(u8*));
	}

//	u8* a = (u8*) malloc(m * k * sizeof(u8));
	u8* a2 = (u8*) malloc(MMAX * KMAX * sizeof(u8));
	u8* a = (u8*) malloc(MMAX * KMAX * sizeof(u8));

	u8* g_tbls = (u8*) malloc(KMAX * TEST_SOURCES * 32 * sizeof(u8));
	u8* g_tbls2 = (u8*) malloc(KMAX * TEST_SOURCES * 32 * sizeof(u8));


//	memcpy(src_err_list, err_list, nerrs);
/*		for (i = 0; i < nerrs; i++)
			src_err_list[i] = i;
		memset(src_in_err, 0, TEST_SOURCES);
		for (i = 0; i < nerrs; i++)
			src_in_err[src_err_list[i]] = 1;
*/

	printf("allocating space for buff...\n");
	// Allocate the arrays
	void* buf;
	for (x = 0; x < stripes2; x++) {
		for (i = k2; i < m2; i++) {
			// @meng: allocate TEST_LEN(chunksize,k) data for each disk
//			if (NULL == (buffs[x][i] = (u8*) malloc(len * sizeof(u8)))) {
			if (posix_memalign(&buf, 64, len2)) {
				printf("alloc error: Fail\n");
				return -1;
			}
			buffs2[x][i] = buf;
		}
	}


	for (x = 0; x < stripes2; x++) {
		for (y = 0; y < m2; y++) {
			for (i = k; i < m; i++) {
				// @meng: allocate TEST_LEN(chunksize,k) data for each disk
//				if (NULL == (buffs[x][i] = (u8*) malloc(len * sizeof(u8)))) {
				if (posix_memalign(&buf, 64, len)) {
					printf("alloc error: Fail\n");
					return -1;
				}
				parities[x][y][i] = buf;
			}
		}
	}

	printf("generating random data...\n");

	printf("generating cauchy matrix...\n");
	gf_gen_cauchy1_matrix(a, m, k);

	gf_gen_cauchy1_matrix(a2, m2, k2);

	double totaltime = 0.0, innertime = 0.0;
	int rounds = 50;
	double totaltime5 = 0.0, innertime5 = 0.0;

	FILE *textfile, *textfile2;
	unsigned char *text, *text2;
	long    numbytes, numbytes2;

	textfile = fopen("/home/cc/1gb-1.bin", "r");
	fseek(textfile, 0L, SEEK_END);
	numbytes = ftell(textfile);
	fseek(textfile, 0L, SEEK_SET);

	text = (u8*)calloc(numbytes, sizeof(u8));
	fread(text, sizeof(char), numbytes, textfile);
	fclose(textfile);

	printf("numbytes:%ld\n", numbytes);
/*
	textfile2 = fopen("/home/cc/1gb-2.bin", "r");
	fseek(textfile2, 0L, SEEK_END);
	numbytes2 = ftell(textfile2);
	fseek(textfile2, 0L, SEEK_SET);

	text2 = (u8*)calloc(numbytes2, sizeof(u8));
	fread(text2, sizeof(char), numbytes, textfile2);
	fclose(textfile2);

	printf("numbytes:%ld\n", numbytes2);
*/


	for (z = 0; z < rounds; z++){
		printf("...new round...\n");
		struct timespec starttime, stop;
		clock_gettime( CLOCK_REALTIME, &starttime);
		{
			int pos = 0;
			for (x = 0; x < stripes2; x++) {
				for (i = 0; i < k2; i++) {
					buffs2[x][i] = &text[pos];
					pos += len2;
				}
			}
			printf("pos:%d\n", pos);
		}

		ec_encode_perf(m, m2, k, k2, a, a2, g_tbls, g_tbls2, buffs2, parities, 
					&start, len, len2, stripes, stripes2, &totaltime);
		clock_gettime( CLOCK_REALTIME, &stop);
		double cost = (stop.tv_sec - starttime.tv_sec)+ (double)( stop.tv_nsec - starttime.tv_nsec )
               		/ (double)BILLION;
		totaltime += cost;

		clock_gettime( CLOCK_REALTIME, &starttime);
		{
			int pos = 0;
			for (x = 0; x < stripes2; x++) {
				for (y = 0; y < m2; y++) {
					pos = 0;
					for (i = 0; i < k; i++) {
						parities[x][y][i] = &buffs2[x][y][pos];
						pos += len;
					}
				}
			}
			printf("pos:%d\n", pos);
		}
		ec_encode_perf_inner(m, m2, k, k2, a, a2, g_tbls, g_tbls2, buffs2, parities, 
					&start, len, len2, stripes, stripes2, &totaltime);
		clock_gettime( CLOCK_REALTIME, &stop);
		cost = (stop.tv_sec - starttime.tv_sec)+ (double)( stop.tv_nsec - starttime.tv_nsec )
               		/ (double)BILLION;
		innertime += cost;

		if (z < 5)
			innertime5 += cost;
		printf( "%lf  stripes:%d  len:%d totaltime5:%lf total time:%lf  innertime5:%lf  innertime:%lf\n",
				cost, stripes, len, totaltime5, totaltime, innertime, innertime5);
	}


	double outerthroughput = ((double)(stripes * stripesize)) * rounds * 1024 / 1000000 / totaltime;
	double outerthroughput2 = ((double)(stripes * stripesize)) * (rounds-5) * 1024 / 1000000 / (totaltime-totaltime5);
	double innerthroughput = ((double)(stripes * stripesize)) * rounds * 1024 / 1000000 / innertime;
	double innerthroughput2 = ((double)(stripes * stripesize)) * (rounds-5) * 1024 / 1000000 / (innertime-innertime5);
//	printf("erasure_code_encode" TEST_TYPE_STR ": ");
	printf("erasure_code_encode" TEST_TYPE_STR " data_num:%d parity_num:%d chunksize:%d : ", k, p, chunksize);
	printf("datasize:%d  totaltime:%lf   outerthroughput:%lfMB/s  totaltime45:%lf  outerthroughput2:%lfMB/s\n",
			stripes * stripesize, totaltime, outerthroughput, totaltime-totaltime5, outerthroughput2);
	printf("datasize:%d  totaltime:%lf   innerthroughput:%lfMB/s  totaltime45:%lf  innerthroughput2:%lfMB/s\n",
			stripes * stripesize, totaltime, innerthroughput, totaltime-totaltime5, innerthroughput2);
	perf_print(start, ((long long)(stripes * stripesize)) * 1024);

	for (x = 0; x < stripes2; x++) {
		for (i = k2; i < m2; i++)
			free(buffs2[x][i]);
		free(buffs2[x]);
	}
	free(buffs2);
	free(a);
	free(g_tbls);


	printf("done all: Pass\n");
	return 0;
}

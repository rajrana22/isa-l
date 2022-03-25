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

void ec_encode_data_stripes(int m, int k, u8 * g_tbls, u8 *** buffs, int len, int stripes)
{
	int x;
	for (x = 0; x < stripes; x++) {
		ec_encode_data(len, k, m - k, g_tbls, buffs[x], &buffs[x][k]);
	}
}

void ec_encode_perf(int m, int k, u8 * a, u8 * g_tbls, u8 *** buffs, struct perf *start, int len, int stripes)
{
	printf("init ec table..\n");
	ec_init_tables(k, m - k, &a[k * k], g_tbls);
	BENCHMARK(start, BENCHMARK_TIME,
		ec_encode_data_stripes(m, k, g_tbls, buffs, len, stripes))
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
	int i, j, m, k, p, x;
	int chunksize, stripes, stripesize, len, datasize;

	// Pick test parameters
	// @meng: this means 10+4 EC
	m = 14;
	k = 10;
	p = 4;
//	const u8 err_list[] = { 2, 4, 5, 7 };
//	const u8 err_list[] = { 1, 2, 3, 4 };

	if (argc < 4)
		printf("./erasure_code_perf_erasure_code_perf data_num parity_num chunksize\n");

	k = atoi(argv[1]);
	p = atoi(argv[2]);
	chunksize = atoi(argv[3]);	// in kb
	len = chunksize * 1024;
	stripesize = len * k;
	datasize = 1024 * 1024 * 1024; //1gib
	stripes = datasize / stripesize;
	m = k + p;
	printf("data_num:%d parity_num:%d m:%d chunksize:%d len:%d stripesize:%d stripes:%d\n",
		k, p, m, chunksize, len, stripesize, stripes);

	printf("erasure_code_perf_erasure_code_perf: %dx%d %d\n", m, len, p);

/*	u8 *temp_buffs[TEST_SOURCES], *buffs[TEST_SOURCES];
	u8 a[MMAX * KMAX];
	u8 g_tbls[KMAX * TEST_SOURCES * 32];//, src_in_err[TEST_SOURCES];
//	u8 src_err_list[TEST_SOURCES];
*/
	struct perf start;

	printf("stripes:%d\n", stripes);

	u8*** buffs = (u8***) malloc(stripes * sizeof(u8**));
	for (x = 0; x < stripes; x++)
		buffs[x] = (u8**) malloc(m * sizeof(u8*));

	u8* a = (u8*) malloc(m * k * sizeof(u8));

	u8* g_tbls = (u8*) malloc(k * p * 32 * sizeof(u8));


//	memcpy(src_err_list, err_list, nerrs);
/*		for (i = 0; i < nerrs; i++)
			src_err_list[i] = i;
		memset(src_in_err, 0, TEST_SOURCES);
		for (i = 0; i < nerrs; i++)
			src_in_err[src_err_list[i]] = 1;
*/

	printf("allocating space for buff...\n");
	// Allocate the arrays
	for (x = 0; x < stripes; x++) {
		for (i = 0; i < m; i++) {
			// @meng: allocate TEST_LEN(chunksize,k) data for each disk
			if (NULL == (buffs[x][i] = (u8*) malloc(len * sizeof(u8)))) {
				printf("alloc error: Fail\n");
				return -1;
			}
		}
	}

	printf("generating random data...\n");

	// Make random data
	// generate a random u8
	for (x = 0; x < stripes; x++)
		for (i = 0; i < k; i++)
			for (j = 0; j < len; j++)
				buffs[x][i][j] = rand();

	printf("generating cauchy matrix...\n");
	gf_gen_cauchy1_matrix(a, m, k);

	// Start encode test
	ec_encode_perf(m, k, a, g_tbls, buffs, &start, len, stripes);
//	printf("erasure_code_encode" TEST_TYPE_STR ": ");
	printf("erasure_code_encode" TEST_TYPE_STR " data_num:%d parity_num:%d chunksize:%d : ", k, p, chunksize);
	printf("datasize:%d\n", stripes * stripesize);
	perf_print(start, (long long)(stripes * stripesize));

/*	// Start decode test
	check = ec_decode_perf(m, k, a, g_tbls, buffs, src_in_err, src_err_list, nerrs,
			       temp_buffs, &start, chunksize);

	if (check == BAD_MATRIX) {
		printf("BAD MATRIX\n");
		return check;
	}

	for (i = 0; i < nerrs; i++) {
		if (0 != memcmp(temp_buffs[i], buffs[src_err_list[i]], TEST_LEN(chunksize,k))) {
			printf("Fail error recovery (%d, %d, %d) - \n", m, k, nerrs);
			// return -1;
		}
	}

//	printf("erasure_code_decode" TEST_TYPE_STR ": ");
	printf("erasure_code_decode" TEST_TYPE_STR " data_num:%d parity_num:%d chunksize:%d : ", k, nerrs, chunksize);
	perf_print(start, (long long)(TEST_LEN(chunksize,k)) * (k));
*/
	for (x = 0; x < stripes; x++) {
		for (i = 0; i < m; i++)
			free(buffs[x][i]);
		free(buffs[x]);
	}
	free(buffs);
	free(a);
	free(g_tbls);


	printf("done all: Pass\n");
	return 0;
}

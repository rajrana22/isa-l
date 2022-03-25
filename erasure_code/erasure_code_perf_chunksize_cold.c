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
#  define TEST_NUM(chunksize) ((GT_L3_CACHE / (chunksize*1024)))
//#  define TEST_NUM(chunksize) 1
#  define TEST_LEN(chunksize, k)  ((chunksize*1024 / k) & ~(64-1))        // k is data units #
#  define TEST_TYPE_STR "_cold"
# else
#  define TEST_TYPE_STR "_cus"
# endif
#endif

#define MMAX TEST_SOURCES
#define KMAX TEST_SOURCES

#define BAD_MATRIX -1

typedef unsigned char u8;

void ec_encode_data_chunks(int m, int k, u8 * g_tbls, u8 *** buffs, int chunksize, int chunks)
{
	int x;
	struct timespec start, stop;
	clock_gettime( CLOCK_REALTIME, &start);
	for (x = 0; x < chunks; x++) {
		ec_encode_data(TEST_LEN(chunksize,k), k, m - k, g_tbls, buffs[x], &buffs[x][k]);
	}
	clock_gettime( CLOCK_REALTIME, &stop);
	double cost = (stop.tv_sec - start.tv_sec)+ (double)( stop.tv_nsec - start.tv_nsec )
               / (double)BILLION;
	printf( "%lf\n", cost);
}

void ec_encode_perf(int m, int k, u8 * a, u8 * g_tbls, u8 *** buffs, struct perf *start, int chunksize, int chunks)
{
	int x;
	for (x = 0; x < chunks; x++)
		ec_init_tables(k, m - k, &a[k * k], g_tbls);
	printf("TEST_LEN(chunksize,k):%d\n", TEST_LEN(chunksize,k));
	BENCHMARK(start, BENCHMARK_TIME,
		ec_encode_data_chunks(m, k, g_tbls, buffs, chunksize, chunks))
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
	int i, j, m, k, nerrs, x;
	int chunksize, chunks;
	void *buf;

	// Pick test parameters
	// @meng: this means 10+4 EC
	m = 14;
	k = 10;
	nerrs = 4;
//	const u8 err_list[] = { 2, 4, 5, 7 };
//	const u8 err_list[] = { 1, 2, 3, 4 };

	if (argc < 4)
		printf("./erasure_code_perf_erasure_code_perf data_num parity_num chunksize\n");

	k = atoi(argv[1]);
	nerrs = atoi(argv[2]);
	chunksize = atoi(argv[3]);
	chunks = TEST_NUM(chunksize);
	m = k + nerrs;
	printf("data_num:%d parity_num:%d m:%d chunksize:%d chunks:%d\n", k, nerrs, m, chunksize, chunks);

	printf("erasure_code_perf_erasure_code_perf: %dx%d %d\n", m, TEST_LEN(chunksize,k), nerrs);

/*	u8 *temp_buffs[TEST_SOURCES], *buffs[TEST_SOURCES];
	u8 a[MMAX * KMAX];
	u8 g_tbls[KMAX * TEST_SOURCES * 32];//, src_in_err[TEST_SOURCES];
//	u8 src_err_list[TEST_SOURCES];
*/
	struct perf start;

	printf("chunks:%d\n", chunks);

	u8*** buffs = (u8***) malloc(chunks * sizeof(u8**));
	for (x = 0; x < chunks; x++)
		buffs[x] = (u8**) malloc(TEST_SOURCES * sizeof(u8*));

	u8* a = (u8*) malloc(MMAX * KMAX * sizeof(u8));

	u8* g_tbls = (u8*) malloc(KMAX * TEST_SOURCES * 32 * sizeof(u8));

	if (m > MMAX || k > KMAX || nerrs > (m - k)) {
		printf(" Input test parameter error\n");
		return -1;
	}

//	memcpy(src_err_list, err_list, nerrs);
/*		for (i = 0; i < nerrs; i++)
			src_err_list[i] = i;
		memset(src_in_err, 0, TEST_SOURCES);
		for (i = 0; i < nerrs; i++)
			src_in_err[src_err_list[i]] = 1;
*/


	// Allocate the arrays
	for (x = 0; x < chunks; x++) {
		for (i = 0; i < m; i++) {
			// @meng: allocate TEST_LEN(chunksize,k) data for each disk
			if (posix_memalign(&buf, 64, TEST_LEN(chunksize, k))) {
				printf("alloc error: Fail\n");
				return -1;
			}
			buffs[x][i] = buf;
		}
	}

	// Make random data
	// generate a random u8
	for (x = 0; x < chunks; x++)
		for (i = 0; i < k; i++)
			for (j = 0; j < TEST_LEN(chunksize,k); j++)
				buffs[x][i][j] = rand();

//		gf_gen_rs_matrix(a[x], m, k);
	gf_gen_cauchy1_matrix(a, m, k);

	// Start encode test
	ec_encode_perf(m, k, a, g_tbls, buffs, &start, chunksize, chunks);
//	printf("erasure_code_encode" TEST_TYPE_STR ": ");
	printf("erasure_code_encode" TEST_TYPE_STR " data_num:%d parity_num:%d chunksize:%d : ", k, nerrs, chunksize);
	printf("datasize:%d\n", TEST_LEN(chunksize,k) * chunks * (k));
	perf_print(start, (long long)(TEST_LEN(chunksize,k)) * chunks * (k));

/*	// Start decode test
	check = ec_decode_perf(m, k, a, g_tbls, buffs, src_in_err, src_err_list, nerrs,
			       temp_buffs, &start, chunksize);

	if (check == BAD_MATRIX) {
		printf("BAD MATRIX\n");
		return check;
	}


//	printf("erasure_code_decode" TEST_TYPE_STR ": ");
	printf("erasure_code_decode" TEST_TYPE_STR " data_num:%d parity_num:%d chunksize:%d : ", k, nerrs, chunksize);
	perf_print(start, (long long)(TEST_LEN(chunksize,k)) * (k));
*/
	for (x = 0; x < chunks; x++) {
		free(buffs[x]);
	}
	free(buffs);
	free(a);
	free(g_tbls);


	printf("done all: Pass\n");
	return 0;
}

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
#include <unistd.h>
#define BILLION  1000000000L;

// #define CACHED_TEST
#ifdef CACHED_TEST
// Cached test, loop many times over small dataset
# define TEST_SOURCES 40
# define TEST_LEN(m_l)  ((128*1024 / m_l) & ~(64-1))
# define TEST_TYPE_STR "_warm"
#else
# ifndef TEST_CUSTOM
// Uncached test.  Pull from large mem base.
#  define TEST_SOURCES 150
#  define GT_L3_CACHE  1024*1024*1024	/* some number > last level cache */
//#  define GT_L3_CACHE  1024*1024	/* some number > last level cache */
#  define TEST_NUM(chunksize_l, k_l) ((GT_L3_CACHE / (chunksize_l*1024*k_l)))
//#  define TEST_NUM(chunksize_l) 1
#  define TEST_LEN(chunksize_l, k_l)  (chunksize_l*1024)        // k_l is data units #
#  define TEST_TYPE_STR "_cold"
# else
#  define TEST_TYPE_STR "_cus"
# endif
#endif

#define MMAX TEST_SOURCES
#define KMAX TEST_SOURCES

#define BAD_MATRIX -1

typedef unsigned char u8;

void ec_encode_data_stripes(int m_l, int m_n, int k_l, int k_n, u8 * g_tbls, u8 * g_tbls2, 
				u8 *** buffs, u8 **** parities, int len_l, int len_n, 
				int stripes_l, int stripes_n, double *t)
{
	int x, y;
	for (x = 0; x < stripes_n; x++) {
		ec_encode_data(len_n, k_n, m_n - k_n, g_tbls2, buffs[x], &buffs[x][k_n]);
		for (y = 0; y < m_n; y++)
			ec_encode_data(len_l, k_l, m_l - k_l, g_tbls, parities[x][y], &parities[x][y][k_l]);
	}
}

void ec_encode_data_stripes_detail(int m_l, int k_l, u8 * g_tbls, u8 *** buffs, int len_l, int stripes_l)
{
	int x;
	struct timespec start, stop;
	for (x = 0; x < stripes_l; x++) {
		clock_gettime( CLOCK_REALTIME, &start);
		ec_encode_data(len_l, k_l, m_l - k_l, g_tbls, buffs[x], &buffs[x][k_l]);
		clock_gettime( CLOCK_REALTIME, &stop);
		double cost = (stop.tv_sec - start.tv_sec)+ (double)( stop.tv_nsec - start.tv_nsec )
               		/ (double)BILLION;
		printf( "%lf  x:%d stripes_l:%d  len_l:%d\n", cost, x, stripes_l, len_l);
	}
}

void ec_encode_perf(int m_l, int m_n, int k_l, int k_n, u8 * a, u8* a2, u8 * g_tbls, u8 * g_tbls2, 
			u8 *** buffs, u8 **** parities, struct perf *start, int len_l, int len_n, 
			int stripes_l, int stripes_n, double* t)
{
	printf("init ec table..\n");
	ec_init_tables(k_l, m_l - k_l, &a[k_l * k_l], g_tbls);
	ec_init_tables(k_n, m_n - k_n, &a2[k_n * k_n], g_tbls2);
	BENCHMARK(start, 0,
		ec_encode_data_stripes(m_l, m_n, k_l, k_n, g_tbls, g_tbls2, buffs, parities, len_l, len_n, 
					stripes_l, stripes_n, t))
}



int ec_decode_perf(int m_l, int k_l, u8 * a, u8 * g_tbls, u8 ** buffs, u8 * src_in_err,
		   u8 * src_err_list, int nerrs, u8 ** temp_buffs, struct perf *start, int chunksize_l)
{
	int i, j, r;
	u8 b[MMAX * KMAX], c[MMAX * KMAX], d[MMAX * KMAX];
	u8 *recov[TEST_SOURCES];

	// Construct b by removing error rows
	for (i = 0, r = 0; i < k_l; i++, r++) {
		while (src_in_err[r])
			r++;
		recov[i] = buffs[r];
		for (j = 0; j < k_l; j++)
			b[k_l * i + j] = a[k_l * r + j];
	}

	if (gf_invert_matrix(b, d, k_l) < 0)
		return BAD_MATRIX;

	for (i = 0; i < nerrs; i++)
		for (j = 0; j < k_l; j++)
			c[k_l * i + j] = d[k_l * src_err_list[i] + j];

	// Recover data
	ec_init_tables(k_l, nerrs, c, g_tbls);
	BENCHMARK(start, BENCHMARK_TIME,
		  ec_encode_data(TEST_LEN(chunksize_l,k_l), k_l, nerrs, g_tbls, recov, temp_buffs));

	return 0;
}

int main(int argc, char *argv[])
{
	int i, x, y, z;
	int m_l, k_l, p_l;
	int k_n,p_n,m_n;
	int chunksize_l, stripes_l, stripesize_l, len_l, datasize;
	int chunksize_n, stripes_n, stripesize_n, len_n;

	/* -------------------------------------------------------------------------- */
	/*                              Argument Parsing                              */
	/* -------------------------------------------------------------------------- */

	// Pick test parameters
	// @meng: this means 10+4 EC
	m_l = 14;
	k_l = 10;
	p_l = 4;

	if (argc < 4)
		printf("USAGE: ./erasure_code_perf_mlec net_data net_parity loc_data loc_parity chunksize_l\n");

	k_n = atoi(argv[1]);
	p_n = atoi(argv[2]);
	k_l = atoi(argv[3]);
	p_l = atoi(argv[4]);
	chunksize_l = atoi(argv[5]);	// in kb

	/* -------------------------------------------------------------------------- */
	/*                            Parameter Conversions                           */
	/* -------------------------------------------------------------------------- */

	chunksize_n = chunksize_l * k_l;
	len_l = chunksize_l * 1024;    // in bytes
	len_n = chunksize_n * 1024;
	stripesize_l = chunksize_l * k_l; // in kb
	stripesize_n = chunksize_n * k_n;
	datasize = 1 * 1024 * 1024; //10 gib in kb
	stripes_n = datasize / stripesize_n;
	stripes_l = datasize / stripesize_l;
	m_l = k_l + p_l;
	m_n = k_n + p_n;
	printf("data_num:%d parity_num:%d m_l:%d chunksize_l:%dKB len_l:%dB stripesize_l:%dKB stripes_l:%d\n",
		k_l, p_l, m_l, chunksize_l, len_l, stripesize_l, stripes_l);
	printf("data_num2:%d parity_num2:%d m_n:%d chunksize_n:%dKB len_n:%dB stripesize_n:%dKB stripes_n:%d\n",
		k_n, p_n, m_n, chunksize_n, len_n, stripesize_n, stripes_n);

	printf("erasure_code_perf_erasure_code_perf: %dx%d %d\n", m_l, len_l, p_l);

	/* -------------------------------------------------------------------------- */
	/*                              Memory Allocation                             */
	/* -------------------------------------------------------------------------- */

	struct perf start;

	printf("stripes_l:%d\n", stripes_l);
	printf("stripes_n:%d\n", stripes_n);

	u8*** buffs2 = (u8***) malloc(stripes_n * sizeof(u8**));
	for (x = 0; x < stripes_n; x++)
		buffs2[x] = (u8**) malloc(m_n * sizeof(u8*));

	u8**** parities = (u8****) malloc(stripes_n * sizeof(u8***));
	for (x = 0; x < stripes_n; x++) {
		parities[x] = (u8***) malloc(m_n * sizeof(u8**));
		for (y = 0; y < m_n; y++)
			parities[x][y] = (u8**) malloc(m_l * sizeof(u8*));
	}

	u8* a2 = (u8*) malloc(MMAX * KMAX * sizeof(u8));
	u8* a = (u8*) malloc(MMAX * KMAX * sizeof(u8));

	u8* g_tbls = (u8*) malloc(KMAX * TEST_SOURCES * 32 * sizeof(u8));
	u8* g_tbls2 = (u8*) malloc(KMAX * TEST_SOURCES * 32 * sizeof(u8));

	printf("allocating space for buff...\n");
	// Allocate the arrays
	void* buf;
	for (x = 0; x < stripes_n; x++) {
		for (i = k_n; i < m_n; i++) {
			// @meng: allocate TEST_LEN(chunksize_l,k_l) data for each disk
			if (posix_memalign(&buf, 64, len_n)) {
				printf("alloc error: Fail\n");
				return -1;
			}
			buffs2[x][i] = buf;
		}
	}


	for (x = 0; x < stripes_n; x++) {
		for (y = 0; y < m_n; y++) {
			for (i = k_l; i < m_l; i++) {
				// @meng: allocate TEST_LEN(chunksize_l,k_l) data for each disk
				if (posix_memalign(&buf, 64, len_l)) {
					printf("alloc error: Fail\n");
					return -1;
				}
				parities[x][y][i] = buf;
			}
		}
	}

	printf("generating random data...\n");

	printf("generating cauchy matrix...\n");
	gf_gen_cauchy1_matrix(a, m_l, k_l);

	gf_gen_cauchy1_matrix(a2, m_n, k_n);

	/* -------------------------------------------------------------------------- */
	/*                           Random File Generation                           */
	/* -------------------------------------------------------------------------- */

	double totaltime = 0.0;
	int rounds = 50;
	double totaltime5 = 0.0;

	FILE *textfile;
	unsigned char *text;
	long    numbytes;
	char fname[] = "1gb-1.bin";

	if (access(fname, F_OK) != 0) {
		// File doesn't exist, create 1 GB file with random data.
		system("dd if=/dev/urandom of=1gb-1.bin bs=1 count=0 seek=1g");
	}
	textfile = fopen(fname, "r");
	fseek(textfile, 0L, SEEK_END);
	numbytes = ftell(textfile);
	fseek(textfile, 0L, SEEK_SET);

	text = (u8*)calloc(numbytes, sizeof(u8));
	fread(text, sizeof(char), numbytes, textfile);
	fclose(textfile);

	printf("numbytes:%ld\n", numbytes);

	/* -------------------------------------------------------------------------- */
	/*                                  Encoding                                  */
	/* -------------------------------------------------------------------------- */

	for (z = 0; z < rounds; z++){
		printf("...new round...\n");
		struct timespec starttime, stop;
		clock_gettime( CLOCK_REALTIME, &starttime);
		{
			int pos = 0;
			for (x = 0; x < stripes_n; x++) {
				for (i = 0; i < k_n; i++) {
					buffs2[x][i] = &text[pos];
					pos += len_n;
				}
			}
			printf("pos:%d\n", pos);
			for (x = 0; x < stripes_n; x++) {
				for (y = 0; y < m_n; y++) {
					pos = 0;
					for (i = 0; i < k_l; i++) {
						parities[x][y][i] = &buffs2[x][y][pos];
						pos += len_l;
					}
				}
			}
			printf("pos:%d\n", pos);
		}

		ec_encode_perf(m_l, m_n, k_l, k_n, a, a2, g_tbls, g_tbls2, buffs2, parities, 
					&start, len_l, len_n, stripes_l, stripes_n, &totaltime);
		clock_gettime( CLOCK_REALTIME, &stop);
		double cost = (stop.tv_sec - starttime.tv_sec)+ (double)( stop.tv_nsec - starttime.tv_nsec )
               		/ (double)BILLION;
		totaltime += cost;
		if (z < 5)
			totaltime5 += cost;
		printf( "%lf  stripes_l:%d  len_l:%d totaltime5:%lf total time:%lf\n", cost, stripes_l, len_l, totaltime5, totaltime);
	}

	/* -------------------------------------------------------------------------- */
	/*                           Throughput Calculation                           */
	/* -------------------------------------------------------------------------- */

	double throughput = ((double)(stripes_l * stripesize_l)) * rounds * 1024 / 1000000 / totaltime;
	double throughput2 = ((double)(stripes_l * stripesize_l)) * (rounds-5) * 1024 / 1000000 / (totaltime-totaltime5);
	printf("erasure_code_encode" TEST_TYPE_STR " data_num:%d parity_num:%d chunksize_l:%d : ", k_l, p_l, chunksize_l);
	printf("datasize:%d  totaltime:%lf   throughput:%lfMB/s  totaltime45:%lf  throughput2:%lfMB/s\n",
			stripes_l * stripesize_l, totaltime, throughput, totaltime-totaltime5, throughput2);
	perf_print(start, ((long long)(stripes_l * stripesize_l)) * 1024);

	/* -------------------------------------------------------------------------- */
	/*                             Memory Deallocation                            */
	/* -------------------------------------------------------------------------- */

	for (x = 0; x < stripes_n; x++) {
		for (i = k_n; i < m_n; i++)
			free(buffs2[x][i]);
		free(buffs2[x]);
	}
	free(buffs2);
	free(a);
	free(g_tbls);


	printf("done all: Pass\n");
	return 0;
}

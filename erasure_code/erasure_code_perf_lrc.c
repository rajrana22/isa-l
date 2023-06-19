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

/* -------------------------------------------------------------------------- */
/*                                  Libraries                                 */
/* -------------------------------------------------------------------------- */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>		// for memset, memcmp
#include "erasure_code.h"
#include "test.h"
#include <time.h>
#include <unistd.h>
#include <math.h>	// for ceil
#define BILLION  1000000000L;

/* -------------------------------------------------------------------------- */
/*                                   Macros                                   */
/* -------------------------------------------------------------------------- */

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
#  define TEST_NUM(chunksize, k_l) ((GT_L3_CACHE / (chunksize*1024*k_l)))
#  define TEST_LEN(chunksize, k_l)  (chunksize*1024)        // k_l is data units #
#  define TEST_TYPE_STR "_cold"
# else
#  define TEST_TYPE_STR "_cus"
# endif
#endif

#define MMAX TEST_SOURCES
#define KMAX TEST_SOURCES

#define BAD_MATRIX -1

typedef unsigned char u8;

/* -------------------------------------------------------------------------- */
/*                                EC Functions                                */
/* -------------------------------------------------------------------------- */

void ec_encode_data_stripes(int local_groups, int m_l, int m_n, int k_l, int k_n, 
				u8 * g_tbls, u8 * g_tbls2, u8 *** net_buffs, u8 **** loc_buffs, 
				int len, int stripes_l, int stripes_n, double *t)
{
	int x, y;
	for (x = 0; x < stripes_n; x++) {
		ec_encode_data(len, k_n, m_n - k_n, g_tbls2, net_buffs[x], &net_buffs[x][k_n]);
		// printf("DEBUG: Global is okay\n");
		for (y = 0; y < local_groups; y++) {
			// printf("DEBUG: x, y: [%d][%d]\n", x, y);
			ec_encode_data(len, k_l, m_l - k_l, g_tbls, loc_buffs[x][y], &loc_buffs[x][y][k_l]);
			// printf("DEBUG: Local is okay\n");
		}
	}
}

void ec_encode_perf(int local_groups, int m_l, int m_n, int k_l, int k_n, u8 * a, u8 * a2, 
			 u8 * g_tbls, u8 * g_tbls2, u8 *** net_buffs, u8 **** loc_buffs, struct perf *start, 
			int len, int stripes_l, int stripes_n, double* t)
{
	// printf("init ec table..\n");
	ec_init_tables(k_l, m_l - k_l, &a[k_l * k_l], g_tbls);
	ec_init_tables(k_n, m_n - k_n, &a2[k_n * k_n], g_tbls2);
	BENCHMARK(start, 0,
		ec_encode_data_stripes(local_groups, m_l, m_n, k_l, k_n, g_tbls, g_tbls2, net_buffs, loc_buffs, len, 
					stripes_l, stripes_n, t))
}

/* --------------------------------- UNUSED --------------------------------- */

void ec_encode_data_stripes_detail(int m_l, int k_l, u8 * g_tbls, u8 *** net_buffs, int len, int stripes_l)
{
	int x;
	struct timespec start, stop;
	for (x = 0; x < stripes_l; x++) {
		clock_gettime( CLOCK_REALTIME, &start);
		ec_encode_data(len, k_l, m_l - k_l, g_tbls, net_buffs[x], &net_buffs[x][k_l]);
		clock_gettime( CLOCK_REALTIME, &stop);
		double cost = (stop.tv_sec - start.tv_sec)+ (double)( stop.tv_nsec - start.tv_nsec )
               		/ (double)BILLION;
		printf( "%lf  x:%d stripes_l:%d  len:%d\n", cost, x, stripes_l, len);
	}
}

int ec_decode_perf(int m_l, int k_l, u8 * a, u8 * g_tbls, u8 ** net_buffs, u8 * src_in_err,
		   u8 * src_err_list, int nerrs, u8 ** temp_buffs, struct perf *start, int chunksize)
{
	int i, j, r;
	u8 b[MMAX * KMAX], c[MMAX * KMAX], d[MMAX * KMAX];
	u8 *recov[TEST_SOURCES];

	// Construct b by removing error rows
	for (i = 0, r = 0; i < k_l; i++, r++) {
		while (src_in_err[r])
			r++;
		recov[i] = net_buffs[r];
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
		  ec_encode_data(TEST_LEN(chunksize,k_l), k_l, nerrs, g_tbls, recov, temp_buffs));

	return 0;
}

/* -------------------------------------------------------------------------- */

int main(int argc, char *argv[])
{
	int global_data, local_groups, global_parity, local_parity;
	int i, x, y, z;
	int m_l, k_l, p_l;
	int m_n, k_n, p_n;
	int chunksize, len, datasize;
	int stripes_l, stripesize_l;
	int stripes_n, stripesize_n;
	int groups;
	int mode = 0;

	/* -------------------------------------------------------------------------- */
	/*                              Argument Parsing                              */
	/* -------------------------------------------------------------------------- */

	if ((argc < 6) || (argc > 7)) {
		printf("-----USAGE-----\n./erasure_code_perf_lrc k l r p chunksize <optional flag> \n");
		printf("\toptional flag: 0 = LRC, 1 = Optimal LRC, default = LRC\n");
		exit(EXIT_FAILURE);
	}
	else {
		global_data = atoi(argv[1]);
		local_groups = atoi(argv[2]);
		global_parity = atoi(argv[3]);
		local_parity = atoi(argv[4]);
		chunksize = atoi(argv[5]);	// in KB

		if (argc == 7) {
			mode = atoi(argv[6]); // 0 for LRC, 1 for Optimal LRC
		}
	}

	/* -------------------------------------------------------------------------- */
	/*                            Parameter Conversions                           */
	/* -------------------------------------------------------------------------- */

	k_n = global_data;
	p_n = global_parity;
	m_n = k_n + p_n;

	if (local_groups == 0) {
		k_l = 0;
	}
	else {
		k_l = (int)ceil(global_data / local_groups);
	}

	total_local_groups = local_groups + (p_n / k_l);

	p_l = local_parity;
	m_l = k_l + p_l;

	len = chunksize * 1024;    // in bytes
	// datasize = 1 * chunksize * k_n;
	datasize = 1 * 1024 * 1024; // 1 GB in KB

	/* Stripe size and number of stripes_l (in KB). */

	stripesize_n = chunksize * k_n;
	stripesize_l = chunksize * k_l;

	stripes_n = datasize / stripesize_n;
	if (stripesize_l == 0) {
		stripes_l = 0;
	}
	else {
		stripes_l = datasize / stripesize_l;
	}


	if (mode == 0) {
		// LRC
		groups = local_groups;
	}
	else if (mode == 1) {
		// Optimal LRC
        groups = local_groups + (p_n / k_l);
	}
	else {
		printf("ERROR: Invalid mode\n");
		exit(EXIT_FAILURE);
	}

	printf("\nChunksize in kilobytes:%dKB\nChunksize in bytes:%dB\n", chunksize, len);

	printf("Data size: %i B = %i KB = %0.2f MB = %0.2f GB\n\n", datasize * 1024, datasize,
		(float)datasize / 1024, (float)datasize / (1024 * 1024));

	printf("-----GLOBAL LAYER-----\n");
	printf("Data Shards:%d\nParity Shards:%d\nTotal Shards:%d\nStripesize:%dKB\nNumber of stripes:%d\n\n",
		k_n, p_n, m_n, stripesize_n, stripes_n);

	printf("-----GROUPS-----\n");
	printf("Local Data Groups: %d\n\n", local_groups);
    if (mode == 1) {
        printf("Local Parity Groups: %d\n", (p_n / k_l));
    }

	printf("-----LOCAL LAYER-----\n");
	printf("Data Shards:%d\nParity Shards:%d\nTotal Shards:%d\nStripesize:%dKB\nNumber of stripes:%d\n\n",
		k_l, p_l, m_l, stripesize_l, stripes_l);

	/* -------------------------------------------------------------------------- */
	/*                              Memory Allocation                             */
	/* -------------------------------------------------------------------------- */

	printf("Allocating space for buffers...\n");

	struct perf start;

	/* Allocating buffer space for stripes. */
	
	u8*** net_buffs = (u8***) malloc(stripes_n * sizeof(u8**));
	for (x = 0; x < stripes_n; x++)
		net_buffs[x] = (u8**) malloc(m_n * sizeof(u8*));

	u8**** loc_buffs = (u8****) malloc(stripes_n * sizeof(u8***));
	for (x = 0; x < stripes_n; x++) {
		loc_buffs[x] = (u8***) malloc(groups * sizeof(u8**));
		for (y = 0; y < groups; y++)
			loc_buffs[x][y] = (u8**) malloc(m_l * sizeof(u8*));
	}

	printf("Size of network buffer: %d stripes x %d chunks x %zu bytes\n",
		stripes_n, m_n, sizeof(u8*));
	printf("Size of local buffer: %d stripes x %d x groups %d chunks x %zu bytes\n",
		stripes_n, local_groups, m_l, sizeof(u8*));

	u8* a2 = (u8*) malloc(MMAX * KMAX * sizeof(u8));
	u8* a = (u8*) malloc(MMAX * KMAX * sizeof(u8));

	u8* g_tbls = (u8*) malloc(KMAX * TEST_SOURCES * 32 * sizeof(u8));
	u8* g_tbls2 = (u8*) malloc(KMAX * TEST_SOURCES * 32 * sizeof(u8));

	// Allocate the arrays
	void* buf;
	for (x = 0; x < stripes_n; x++) {
		for (i = k_n; i < m_n; i++) {
			// @meng: allocate TEST_LEN(chunksize,k_l) data for each disk
			if (posix_memalign(&buf, 64, len)) {
				printf("Error: Allocation Failure\n");
				return -1;
			}
			net_buffs[x][i] = buf;
		}
	}

	for (x = 0; x < stripes_n; x++) {
		for (y = 0; y < groups; y++) {
			for (i = k_l; i < m_l; i++) {
				// @meng: allocate TEST_LEN(chunksize,k_l) data for each disk
				if (posix_memalign(&buf, 64, len)) {
					printf("Error: Allocation Failure\n");
					return -1;
				}
				loc_buffs[x][y][i] = buf;
			}
		}
	}

	printf("Generating cauchy matrix...\n");

	gf_gen_cauchy1_matrix(a, m_l, k_l);
	gf_gen_cauchy1_matrix(a2, m_n, k_n);

	/* -------------------------------------------------------------------------- */
	/*                           Random File Generation                           */
	/* -------------------------------------------------------------------------- */

	printf("Generating random data...\n");

	double totaltime = 0.0;
	int rounds = 50;
	double totaltime5 = 0.0;

	FILE *textfile;
	unsigned char *text;
	long    numbytes;
	char fname[] = "1gb-1.bin";

	if (access(fname, F_OK) != 0) {
		// File doesn't exist, create 1 GB file with random data.
		system("dd if=/dev/urandom of=1gb-1.bin bs=1 count=0 seek=1G");
	}
	textfile = fopen(fname, "r");
	fseek(textfile, 0L, SEEK_END);
	numbytes = ftell(textfile);
	fseek(textfile, 0L, SEEK_SET);

	text = (u8*)calloc(numbytes, sizeof(u8));
	fread(text, sizeof(char), numbytes, textfile);
	fclose(textfile);

	printf("Number of bytes in file: %ld GB\n", numbytes / 1024 / 1024 / 1024);

	/* -------------------------------------------------------------------------- */
	/*                                  Encoding                                  */
	/* -------------------------------------------------------------------------- */

	int groups;

	if (mode == 0) {
		// LRC
		groups = local_groups;
	}
	else if (mode == 1) {
		// Optimal LRC
		groups = total_local_groups;
	}
	else {
		printf("ERROR: Invalid mode\n");
		exit(EXIT_FAILURE);
	}

	printf("Encoding...\n");

	for (z = 0; z < rounds; z++){
		// printf("...New Round...\n");
		struct timespec starttime, stop;
		clock_gettime( CLOCK_REALTIME, &starttime);
		{
			int pos = 0;
			for (x = 0; x < stripes_n; x++) {
				for (i = 0; i < k_n; i++) {
					net_buffs[x][i] = &text[pos];
					pos += len;
				}
			}
			// printf("pos:%d\n", pos);
            // k_n = 8
            // groups = 2
            // k_l = 4

			for (x = 0; x < stripes_n; x++) {
				for (y = 0; y < groups; y++) {
					pos = 0;
					for (i = 0; i < k_l; i++) {
                        // if (z == 0 && x == 0) {
                        //     printf("loc_buffs index: [%d][%d]\n", y, i);
                        //     printf("net_buffs index: %d\n", i + (y * k_l));
                        // }
						loc_buffs[x][y][i] = &net_buffs[x][i + (y * k_l)][pos];
						pos += len;
					}
				}
			}
		}

		ec_encode_perf(groups, m_l, m_n, k_l, k_n, a, a2, g_tbls, g_tbls2, net_buffs, loc_buffs, 
					&start, len, stripes_l, stripes_n, &totaltime);
		clock_gettime( CLOCK_REALTIME, &stop);
		double cost = (stop.tv_sec - starttime.tv_sec)+ (double)( stop.tv_nsec - starttime.tv_nsec )
               		/ (double)BILLION;
		totaltime += cost;
		if (z < 5)
			totaltime5 += cost;
		// printf( "%lf  stripes:%d  len:%d totaltime5:%lf total time:%lf\n", cost, stripes_n, len, totaltime5, totaltime);
		printf("Time Elapsed: %lf\n", totaltime);
	}

	/* -------------------------------------------------------------------------- */
	/*                           Throughput Calculation                           */
	/* -------------------------------------------------------------------------- */

    /* Deprecated */
	// double throughput = ((double)(stripes_n * stripesize_n)) * rounds * 1024 / 1000000 / totaltime;

	double throughput = ((double)(stripes_n * stripesize_n)) * (rounds-5) * 1024 / 1000000 / (totaltime-totaltime5);
    printf("Bytes Encoded: %d B\n", stripes_n * stripesize_n);
    printf("Time Taken: %lf s\n", (totaltime-totaltime5));
    printf("Overall Throughput: %lf MB/s\n", throughput);

	/* -------------------------------------------------------------------------- */
	/*                             Memory Deallocation                            */
	/* -------------------------------------------------------------------------- */

	for (x = 0; x < stripes_n; x++) {
		for (i = k_n; i < m_n; i++) {
			free(net_buffs[x][i]);
		}
		free(net_buffs[x]);
	}
	free(net_buffs);

	for (x = 0; x < stripes_n; x++) {
		for (y = 0; y < groups; y++) {
			for (i = k_l; i < m_l; i++) {
				free(loc_buffs[x][y][i]);
			}
			free(loc_buffs[x][y]);
		}
		free(loc_buffs[x]);
	}
	free(loc_buffs);

	free(a);
	free(a2);

	free(g_tbls);
	free(g_tbls2);

	printf("Done all: Pass\n");
	return 0;
}

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

void ec_encode_data_stripes(int m_l1, int m_l2, int m_n, int k_l1, int k_l2, int k_n, u8 * g_tbls, u8 * g_tbls2, u8 * g_tbls3,
                            u8 *** net_buffs, u8 *** loc_buffs1, u8 *** loc_buffs2, int len, int num_stripes_n)
{
	int x, y;
	for (x = 0; x < num_stripes_n; x++) {
		ec_encode_data(len, k_n, m_n - k_n, g_tbls2, net_buffs[x], &net_buffs[x][k_n]);
        ec_encode_data(len, k_l1, m_l1 - k_l1, g_tbls, loc_buffs1[x], &loc_buffs1[x][k_l1]);
        ec_encode_data(len, k_l2, m_l2 - k_l2, g_tbls3, loc_buffs2[x], &loc_buffs2[x][k_l2]);
	}
}

// num_stripes_l = num_stripes_n * local_groups

void ec_encode_perf(int m_l1, int m_l2, int m_n, int k_l1, int k_l2, int k_n, u8 * a, u8 * a2, u8 * a3, 
			 u8 * g_tbls, u8 * g_tbls2, u8 * g_tbls3, u8 *** net_buffs, u8 *** loc_buffs1, u8 *** loc_buffs2, struct perf *start, 
			int len, int num_stripes_n)
{
	// printf("init ec table..\n");
	ec_init_tables(k_l1, m_l1 - k_l1, &a[k_l1 * k_l1], g_tbls);
	ec_init_tables(k_n, m_n - k_n, &a2[k_n * k_n], g_tbls2);
    ec_init_tables(k_l2, m_l2 - k_l2, &a3[k_l2 * k_l2], g_tbls3);
	BENCHMARK(start, 0,
		ec_encode_data_stripes(m_l1, m_l2, m_n, k_l1, k_l2, k_n, g_tbls, g_tbls2, g_tbls3, net_buffs, loc_buffs1, loc_buffs2, len, num_stripes_n))
}

/* -------------------------------------------------------------------------- */

int main(int argc, char *argv[])
{
	int global_data, local_groups, global_parity, local_parity;
	int i, x, y, z;
	int m_l1, m_l2, k_l1, k_l2, p_l;
	int m_n, k_n, p_n;
	int chunksize, len, datasize;
	int num_stripes_n, stripesize_n;
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
		k_l1 = 0;
        k_l2 = 0;
	}
	else {
		// First local group's number of local data chunks
        k_l1 = (int)ceil(global_data / local_groups);

        // Second local group's number of local data chunks
        k_l2 = (global_data - 1) / local_groups;
	}

    p_l = local_parity;

    // Total local parity chunks for each local group
    m_l1 = k_l1 + p_l;
    m_l2 = k_l2 + p_l;

	len = chunksize * 1024;    // in bytes
	// datasize = 1 * chunksize * k_n;
	datasize = 1 * 1024 * 1024; // 1 GB in KB

	/* Stripe size and number of num_stripes_l (in KB). */

	stripesize_n = chunksize * k_n;

    num_stripes_n = datasize / stripesize_n;

	if (mode == 0) {
		// LRC
		groups = local_groups;
	}
	else if (mode == 1) {
		// Optimal LRC
        groups = local_groups + (p_n / k_l1);
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
		k_n, p_n, m_n, stripesize_n, num_stripes_n);

	// printf("-----GROUPS-----\n");
	// printf("Local Data Groups: %d\n\n", local_groups);
    // if (mode == 1) {
    //     printf("Local Parity Groups: %d\n", (p_n / k_l));
    // }

	// printf("-----LOCAL LAYER-----\n");
	// printf("Data Shards:%d\nParity Shards:%d\nTotal Shards:%d\nStripesize:%dKB\nNumber of stripes:%d\n\n",
	// 	k_l, p_l, m_l, stripesize_l, num_stripes_l);

	/* -------------------------------------------------------------------------- */
	/*                              Memory Allocation                             */
	/* -------------------------------------------------------------------------- */

	printf("Allocating space for buffers...\n");

	struct perf start;

	/* Allocating buffer space for stripes. */
	
	u8*** net_buffs = (u8***) malloc(num_stripes_n * sizeof(u8**));
	for (x = 0; x < num_stripes_n; x++)
		net_buffs[x] = (u8**) malloc(m_n * sizeof(u8*));

    // Each network stripe has m_l1 + m_l2 local chunks
    u8*** loc_buffs1 = (u8***) malloc(num_stripes_n * sizeof(u8**));
	for (x = 0; x < num_stripes_n; x++)
		loc_buffs1[x] = (u8**) malloc(m_l1 * sizeof(u8*));

    u8*** loc_buffs2 = (u8***) malloc(num_stripes_n * sizeof(u8**));
	for (x = 0; x < num_stripes_n; x++)
		loc_buffs2[x] = (u8**) malloc(m_l2 * sizeof(u8*));

	printf("Size of network buffer: %d stripes x %d chunks x %zu bytes\n",
		num_stripes_n, m_n, sizeof(u8*));
	printf("Size of local buffer 1: %d stripes x %d chunks x %zu bytes\n",
		num_stripes_n, m_l1, sizeof(u8*));
    printf("Size of local buffer 2: %d stripes x %d chunks x %zu bytes\n",
		num_stripes_n, m_l2, sizeof(u8*));

    u8* a = (u8*) malloc(MMAX * KMAX * sizeof(u8));
	u8* a2 = (u8*) malloc(MMAX * KMAX * sizeof(u8));
    u8* a3 = (u8*) malloc(MMAX * KMAX * sizeof(u8));

	u8* g_tbls = (u8*) malloc(KMAX * TEST_SOURCES * 32 * sizeof(u8));
	u8* g_tbls2 = (u8*) malloc(KMAX * TEST_SOURCES * 32 * sizeof(u8));
    u8* g_tbls3 = (u8*) malloc(KMAX * TEST_SOURCES * 32 * sizeof(u8));

	// Allocate the arrays
	void* buf;
	for (x = 0; x < num_stripes_n; x++) {
		for (i = k_n; i < m_n; i++) {
			if (posix_memalign(&buf, 64, len)) {
				printf("Error: Allocation Failure\n");
				return -1;
			}
			net_buffs[x][i] = buf;
		}
	}

    for (x = 0; x < num_stripes_n; x++) {
		for (i = k_l1; i < m_l1; i++) {
			if (posix_memalign(&buf, 64, len)) {
				printf("Error: Allocation Failure\n");
				return -1;
			}
			loc_buffs1[x][i] = buf;
		}
	}

    for (x = 0; x < num_stripes_n; x++) {
		for (i = k_l2; i < m_l2; i++) {
			if (posix_memalign(&buf, 64, len)) {
				printf("Error: Allocation Failure\n");
				return -1;
			}
			loc_buffs2[x][i] = buf;
		}
	}

	printf("Generating cauchy matrix...\n");

	gf_gen_cauchy1_matrix(a, m_l1, k_l1);
	gf_gen_cauchy1_matrix(a2, m_n, k_n);
    gf_gen_cauchy1_matrix(a3, m_l2, k_l2);

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

	printf("Encoding...\n");

	for (z = 0; z < rounds; z++){
		// printf("...New Round...\n");
		struct timespec starttime, stop;
		clock_gettime( CLOCK_REALTIME, &starttime);
		{
			int pos = 0;
			for (x = 0; x < num_stripes_n; x++) {
				for (i = 0; i < k_n; i++) {
					net_buffs[x][i] = &text[pos];
					pos += len;
				}
			}
			// printf("pos:%d\n", pos);
            // k_n = 8
            // groups = 2
            // k_l = 4

            pos = 0;
            for (x = 0; x < num_stripes_n; x++) {
				for (i = 0; i < k_l1; i++) {
					loc_buffs1[x][i] = &text[pos];
					pos += len;
				}
                for (i = 0; i < k_l2; i++) {
                    loc_buffs2[x][i] = &text[pos];
                    pos += len;
                }
			}
		}

		ec_encode_perf(m_l1, m_l2, m_n, k_l1, k_l2, k_n, a, a2, a3, g_tbls, g_tbls2, g_tbls3, net_buffs, loc_buffs1, loc_buffs2, 
					&start, len, num_stripes_n);
		clock_gettime( CLOCK_REALTIME, &stop);
		double cost = (stop.tv_sec - starttime.tv_sec)+ (double)( stop.tv_nsec - starttime.tv_nsec )
               		/ (double)BILLION;
		totaltime += cost;
		if (z < 5)
			totaltime5 += cost;
		// printf( "%lf  stripes:%d  len:%d totaltime5:%lf total time:%lf\n", cost, num_stripes_n, len, totaltime5, totaltime);
		printf("Time Elapsed: %lf\n", totaltime);
	}

	/* -------------------------------------------------------------------------- */
	/*                           Throughput Calculation                           */
	/* -------------------------------------------------------------------------- */

    /* Deprecated */
	// double throughput = ((double)(num_stripes_n * stripesize_n)) * rounds * 1024 / 1000000 / totaltime;

	double throughput = ((double)(num_stripes_n * stripesize_n)) * (rounds-5) * 1024 / 1000000 / (totaltime-totaltime5);
    printf("Bytes Encoded: %d B\n", num_stripes_n * stripesize_n);
    printf("Time Taken: %lf s\n", (totaltime-totaltime5));
    printf("Overall Throughput: %lf MB/s\n", throughput);

	/* -------------------------------------------------------------------------- */
	/*                             Memory Deallocation                            */
	/* -------------------------------------------------------------------------- */

	for (x = 0; x < num_stripes_n; x++) {
		for (i = k_n; i < m_n; i++) {
			free(net_buffs[x][i]);
		}
		free(net_buffs[x]);
	}
	free(net_buffs);

	for (x = 0; x < num_stripes_n; x++) {
        for (i = k_l1; i < m_l1; i++) {
            free(loc_buffs1[x][i]);
        }
		free(loc_buffs1[x]);
	}
	free(loc_buffs1);

    for (x = 0; x < num_stripes_n; x++) {
        for (i = k_l2; i < m_l2; i++) {
            free(loc_buffs2[x][i]);
        }
		free(loc_buffs2[x]);
	}
	free(loc_buffs2);

	free(a);
	free(a2);

	free(g_tbls);
	free(g_tbls2);

	printf("Done all: Pass\n");
	return 0;
}

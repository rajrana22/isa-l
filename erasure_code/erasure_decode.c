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
#include <string.h> // for memset, memcmp
#include "erasure_code.h"
#include "test.h"
#include <time.h>
#include <unistd.h>
#define BILLION 1000000000L;

// #define CACHED_TEST
#ifdef CACHED_TEST
// Cached test, loop many times over small dataset
#define TEST_SOURCES 40
#define TEST_LEN(m) ((128 * 1024 / m) & ~(64 - 1))
#define TEST_TYPE_STR "_warm"
#else
#ifndef TEST_CUSTOM
// Uncached test.  Pull from large mem base.
#define TEST_SOURCES 150
#define GT_L3_CACHE 1024 * 1024 * 1024 /* some number > last level cache */
//#  define GT_L3_CACHE  1024*1024	/* some number > last level cache */
#define TEST_NUM(chunksize, k) ((GT_L3_CACHE / (chunksize * 1024 * k)))
//#  define TEST_NUM(chunksize) 1
#define TEST_LEN(chunksize, k) (chunksize * 1024) // k is data units #
#define TEST_TYPE_STR "_cold"
#else
#define TEST_TYPE_STR "_cus"
#endif
#endif

#define MMAX TEST_SOURCES
#define KMAX TEST_SOURCES

#define BAD_MATRIX -1

typedef unsigned char u8;

void ec_encode_data_stripes(int m, int k, u8 *g_tbls, u8 ***buffs, int len, int stripes, double *t)
{
    int x;
    for (x = 0; x < stripes; x++)
    {
        ec_encode_data(len, k, m - k, g_tbls, buffs[x], &buffs[x][k]);
    }
}

void ec_encode_data_stripes_detail(int m, int k, u8 *g_tbls, u8 ***buffs, int len, int stripes)
{
    int x;
    struct timespec start, stop;
    for (x = 0; x < stripes; x++)
    {
        clock_gettime(CLOCK_REALTIME, &start);
        ec_encode_data(len, k, m - k, g_tbls, buffs[x], &buffs[x][k]);
        clock_gettime(CLOCK_REALTIME, &stop);
        double cost = (stop.tv_sec - start.tv_sec) + (double)(stop.tv_nsec - start.tv_nsec) / (double)BILLION;
        printf("%lf  x:%d stripes:%d  len:%d\n", cost, x, stripes, len);
    }
}

void ec_encode_perf(int m, int k, u8 *a, u8 *g_tbls, u8 ***buffs, struct perf *start, int len, int stripes, double *t)
{
    printf("init ec table..\n");
    ec_init_tables(k, m - k, &a[k * k], g_tbls);
    BENCHMARK(start, 0,
              ec_encode_data_stripes(m, k, g_tbls, buffs, len, stripes, t))
}

int ec_decode_stripe(int m, int k, u8 *a, u8 *g_tbls, u8 **buffs, int len, u8 *src_in_err, u8 *src_err_list, int nerrs)
{
    int i, j, r;
    u8 *b = (u8 *)malloc(sizeof(u8) * MMAX * KMAX);
    u8 *c = (u8 *)malloc(sizeof(u8) * MMAX * KMAX);
    u8 *d = (u8 *)malloc(sizeof(u8) * MMAX * KMAX);
    u8 *recov[k];

    // Construct b by removing error rows
    for (i = 0, r = 0; i < k; i++, r++)
    {
        while (src_in_err[r])
            r++;
        recov[i] = buffs[r];
        for (j = 0; j < k; j++)
            b[k * i + j] = a[k * r + j];
    }

    if (gf_invert_matrix(b, d, k) < 0)
        return BAD_MATRIX;

    free(b);

    for (i = 0; i < nerrs; i++)
        for (j = 0; j < k; j++)
            c[k * i + j] = d[k * src_err_list[i] + j];

    // Recover data
    ec_init_tables(k, nerrs, c, g_tbls);
    ec_encode_data(len, k, nerrs, g_tbls, recov, buffs);

    free(c);
    free(d);

    return 0;
}

void ec_decode_perf(int m, int k, u8 **a, u8 *g_tbls, u8 ***buffs, int len, int stripes,
                    u8 **tot_src_in_err, u8 **tot_src_err_list, int nerrs)
{
    // Loop through stripes in data
    for (int x = 0; x < stripes; x++)
    {
        // Decode a single stripe of data
        int ret = ec_decode_stripe(m, k, a[x], g_tbls, buffs[x], len, tot_src_in_err[x], tot_src_err_list[x], nerrs);
        if (ret < 0) {
            printf("ERROR: Decoding failure\n");
            return;
        }
    }
}

int main(int argc, char *argv[])
{

    /* -------------------------------------------------------------------------- */
    /*                              Argument Parsing                              */
    /* -------------------------------------------------------------------------- */

    int i, m, k, p, x;
    int chunksize, stripes, stripesize, len, datasize;

    // Pick test parameters
    // @meng: this means 10+4 EC
    m = 14;
    k = 10;
    p = 4;

    if (argc < 4)
        printf("./erasure_code_perf_erasure_code_perf data_num parity_num chunksize\n");

    k = atoi(argv[1]);
    p = atoi(argv[2]);
    chunksize = atoi(argv[3]);  // in kb
    len = chunksize * 1024;     // in bytes
    stripesize = chunksize * k; // in kb
    datasize = 1 * 1024 * 1024; // 1 gib in kb
    stripes = datasize / stripesize;
    m = k + p;
    printf("data_num:%d parity_num:%d m:%d chunksize:%dKB len:%dB stripesize:%dKB stripes:%d\n",
           k, p, m, chunksize, len, stripesize, stripes);

    printf("erasure_code_perf_erasure_code_perf: %dx%d %d\n", m, len, p);

    /* -------------------------------------------------------------------------- */
    /*                              Memory Allocation                             */
    /* -------------------------------------------------------------------------- */

    struct perf start;

    printf("stripes:%d\n", stripes);

    // Allocating buffer space for stripes (data and parity shards).
    u8 ***buffs = (u8 ***)malloc(stripes * sizeof(u8 **));
    for (x = 0; x < stripes; x++)
        buffs[x] = (u8 **)malloc(m * sizeof(u8 *));

    // Needed for encoding and the cauchy matrix.
    u8 **a = (u8 **)malloc(stripes * sizeof(u8 *));
    for (x = 0; x < stripes; x++) {
        a[x] = (u8 *)malloc(MMAX * KMAX * sizeof(u8));
    }

    // Precomputed matrix.
    u8 *g_tbls = (u8 *)malloc(KMAX * TEST_SOURCES * 32 * sizeof(u8));

    printf("allocating space for buff...\n");
    // Allocate the arrays
    void *buf;
    for (x = 0; x < stripes; x++)
    {
        for (i = k; i < m; i++)
        {
            // @meng: allocate TEST_LEN(chunksize,k) data for each disk
            if (posix_memalign(&buf, 64, len))
            {
                printf("alloc error: Fail\n");
                return -1;
            }
            buffs[x][i] = buf;
        }
    }

    // Create errors in sources
    int nerrs = p;
    
    u8 tot_src_in_err[stripes][MMAX];
    u8 tot_src_err_list[stripes][MMAX];

    memset(tot_src_in_err, 0, sizeof(tot_src_in_err));

    for (x = 0; x < stripes; x++) {
        for (i = 0; i < nerrs; i++) {
            tot_src_err_list[x][i] = rand() % (k + p);
            tot_src_in_err[x][tot_src_err_list[x][i]] = 1;
        }
    }

    printf("generating random data...\n");

    printf("generating cauchy matrices...\n");
    for (x = 0; x < stripes; x++)
        gf_gen_cauchy1_matrix(a[x], m, k);

    /* -------------------------------------------------------------------------- */
    /*                           Random File Generation                           */
    /* -------------------------------------------------------------------------- */

    double totaltime = 0.0;
    int rounds = 50;
    double totaltime5 = 0.0;

    FILE *textfile;
    unsigned char *text;
    long numbytes;
    char fname[] = "1gb-1.bin";

    if (access(fname, F_OK) != 0)
    {
        // File doesn't exist, create 16 GB file with random data.
        system("dd if=/dev/urandom of=1gb-1.bin bs=1 count=0 seek=1G");
    }
    textfile = fopen(fname, "r");
    fseek(textfile, 0L, SEEK_END);
    numbytes = ftell(textfile);
    fseek(textfile, 0L, SEEK_SET);

    text = (u8 *)calloc(numbytes, sizeof(u8));
    fread(text, sizeof(char), numbytes, textfile);
    fclose(textfile);

    printf("numbytes:%ld\n", numbytes);

    /* -------------------------------------------------------------------------- */
    /*                                  Encoding                                  */
    /* -------------------------------------------------------------------------- */

    for (int y = 0; y < rounds; y++)
    {
        printf("...new round...\n");
        struct timespec starttime, stop;
        clock_gettime(CLOCK_REALTIME, &starttime);
        {
            int pos = 0;
            for (x = 0; x < stripes; x++)
                for (i = 0; i < k; i++)
                {
                    buffs[x][i] = &text[pos];
                    pos += len;
                }
            //		printf("pos:%d\n", pos);
        }
        ec_decode_perf(m, k, a, g_tbls, buffs, len, stripes, tot_src_in_err, tot_src_err_list, nerrs);
        clock_gettime(CLOCK_REALTIME, &stop);
        double cost = (stop.tv_sec - starttime.tv_sec) + (double)(stop.tv_nsec - starttime.tv_nsec) / (double)BILLION;
        totaltime += cost;
        if (y < 5)
            totaltime5 += cost;
        printf("%lf  stripes:%d  len:%d totaltime5:%lf total time:%lf\n", cost, stripes, len, totaltime5, totaltime);
    }

    /* -------------------------------------------------------------------------- */
    /*                           Throughput Calculation                           */
    /* -------------------------------------------------------------------------- */

    double throughput = ((double)(stripes * stripesize)) * rounds * 1024 / 1000000 / totaltime;
    double throughput2 = ((double)(stripes * stripesize)) * (rounds - 5) * 1024 / 1000000 / (totaltime - totaltime5);
    printf("erasure_code_encode" TEST_TYPE_STR " data_num:%d parity_num:%d chunksize:%d : ", k, p, chunksize);
    printf("datasize:%d  totaltime:%lf   throughput:%lfMB/s  totaltime45:%lf  throughput2:%lfMB/s\n",
           stripes * stripesize, totaltime, throughput, totaltime - totaltime5, throughput2);
    perf_print(start, ((long long)(stripes * stripesize)) * 1024);
    printf("Overall Throughput: %lf MB/s\n", throughput2);

    /* -------------------------------------------------------------------------- */
    /*                             Memory Deallocation                            */
    /* -------------------------------------------------------------------------- */

    for (x = 0; x < stripes; x++)
    {
        for (i = k; i < m; i++)
            free(buffs[x][i]);
        free(buffs[x]);
    }
    free(buffs);
    free(a);
    free(g_tbls);

    printf("done all: Pass\n");
    return 0;
}

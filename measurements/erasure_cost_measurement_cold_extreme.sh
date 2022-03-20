#!/bin/bash
for n in {27..50}
do
  for k in {1..10}
  do
    erasure_code/erasure_code_perf_chunksize_cold $n  $k 128
  done
done

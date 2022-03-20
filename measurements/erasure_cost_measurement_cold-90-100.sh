#!/bin/bash
for n in {90..100}
do
  for k in {1..10}
  do
    erasure_code/erasure_code_perf_chunksize_cold $n  $k 128
  done
done

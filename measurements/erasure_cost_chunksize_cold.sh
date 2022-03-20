#!/bin/bash


benchmark(){
  n=$1
  k=$2
  for chunksize in {1,2,4,8,16,32,64,128,256,512,1024}
  do
    erasure_code/erasure_code_perf_chunksize_cold $n  $k $chunksize
    erasure_code/erasure_code_perf_chunksize_cold $n  $k $(($chunksize*1024))
  done
}

benchmark 8 1
benchmark 8 2
benchmark 8 3
benchmark 8 4
benchmark 4 1
benchmark 4 2
benchmark 16 1
benchmark 16 2
benchmark 16 3
benchmark 16 4
benchmark 16 5
benchmark 16 6
benchmark 16 7
benchmark 16 8

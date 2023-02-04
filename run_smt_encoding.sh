#!/bin/bash

result=`/opt/dreal/4.21.06.2/bin/dreal --model test_smt_encoding.smt2`
# echo $result
if [[ "$result" == *"delta-sat"* ]]; then
	output=1
else
	output=0
fi
# echo $output

exit $output
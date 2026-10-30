#!/bin/bash
# =============================================
# 创建符合 "SystemC建模需求" 的目录结构
# =============================================

DOC_DIR=doc
INC_DIR=include
JSON_DIR=json
SRC_DIR=source
TEST_DIR=test

TARGET=$1

if [ -z "$TARGET" ]; then
    echo "Usage: $0 <path>"
    exit 1
fi

DOC_DIR=$TARGET/$DOC_DIR
INC_DIR=$TARGET/$INC_DIR
JSON_DIR=$TARGET/$JSON_DIR
SRC_DIR=$TARGET/$SRC_DIR
TEST_DIR=$TARGET/$TEST_DIR

for dir in $DOC_DIR $INC_DIR $JSON_DIR $SRC_DIR $TEST_DIR; do
    mkdir -p $dir
done
#!/bin/bash
protoc -I="." --python_out="./" *.proto --pyi_out="./"

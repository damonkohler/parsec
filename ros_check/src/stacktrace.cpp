// Copyright 2011 Google Inc.
// Author: moesenle@google.com (Lorenz Moesenlechner)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <stdlib.h>
#include <execinfo.h>

namespace ros_check {

void PrintStacktrace(FILE *stream, int skip) {
  void *array[30];
  size_t size;
  char **strings;
  size_t i;
     
  size = backtrace(array, 30);
  strings = backtrace_symbols(array, size);
  fprintf(stream, "Stack frames:\n");
  // Skip the first frame always because we that will always be
  // PrintStacktrace.
  for (i = skip + 1; i < size; i++) {
    fprintf(stream, "%s\n", strings[i]);
  }

  free(strings);
}

void PrintStacktraceAndDie(FILE *stream) {
  PrintStacktrace(stream, 1);
  abort();
}

}

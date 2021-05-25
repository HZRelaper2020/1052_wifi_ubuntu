////////////////////////////////////////////////////////////////////////////
//                           **** WAVPACK ****                            //
//                  Hybrid Lossless Wavefile Compressor                   //
//              Copyright (c) 1998 - 2013 Conifer Software.               //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

// decorr_tables.c

// These four tables specify the characteristics of the decorrelation filters
// for the four basic compression modes (fast, normal, high, and very high).
//
// The first entry in the table represents the "default" filter for the
// corresponding mode; subsequent entries represent filters that are tried
// in the "extra" modes 1-3 ("extra" modes 4-6 create filters from scratch).
// The number of entries for each table must be a power of two, and this is
// not checked anywhere, so be careful changing the lengths!
//
// The first value indicates whether the filter is applied to joint stereo
// data (0=L/R, 1=M/S) and the second value represents the "delta" value of
// the adaptive filter. The rest of the values (2-16, depending on mode) are
// the "terms" of the filter.
//
// Each term represents one layer of the sequential filter, where positive
// values indicate the relative sample involved from the same channel (1=prev),
// 17 & 18 are special functions using the previous 2 samples, and negative
// values indicate cross channel decorrelation (in stereo only).
//
// It would be ideal if this was the only source for the decorrelation tables,
// but unfortunately the defaults (first entry) are duplicated in the assembly
// code for the function pack_decorr_mono_buffer(). There is a check in there
// to verify that the hardcoded filter is passed, and if not a general version
// is employed. But obviously some of the assembly advantage is lost in that
// case, so if these default filters change it's a good idea to change the
// assembly versions.

#include "wavpack_local.h"

static const WavpackDecorrSpec fast_specs [] = {
        { 1, 2, {18,17}},  // 0
        { 1, 1, {17,17}},  // 1
        { 0, 2, {18,17}},  // 2
        { 0, 1, {17,17}},  // 3
        { 1, 3, { 1,18}},  // 4
        { 1, 1, {17, 1}},  // 5
        { 0, 1, { 1,17}},  // 6
        { 0, 1, {-2,17}},  // 7
        { 0, 2, {-1,17}},  // 8
        { 1, 1, {17, 2}},  // 9
        { 0, 3, {18,18}},  // 10
        { 0, 1, {17, 1}},  // 11
        { 1, 6, { 1, 2}},  // 12
        { 1, 1, {17, 3}},  // 13
        { 0, 1, {-2, 3}},  // 14
        { 0, 1, { 2,17}},  // 15
        { 0, 1, {18,-2}},  // 16
        { 0, 1, {-1,17}},  // 17
        { 0, 1, {18,17}},  // 18
        { 0, 1, {17, 2}},  // 19
        { 1, 2, {18,-2}},  // 20
        { 1, 1, { 1,17}},  // 21
        { 0, 3, {18, 2}},  // 22
        { 0, 1, {17,-2}},  // 23
        { 0, 1, {18,-2}},  // 24
        { 1, 2, {17,-3}},  // 25
        { 0, 1, {18, 3}},  // 26
        { 0, 1, {18,18}},  // 27
        { 1, 1, { 1, 3}},  // 28
        { 1, 1, {18, 3}},  // 29
        { 1, 1, { 1, 3}},  // 30
        { 0, 2, {18,17}},  // 31
        { 1, 1, { 1,17}},  // 32
        { 1, 1, {17, 3}},  // 33
        { 0, 3, {18,17}},  // 34
        { 0, 1, {18,18}},  // 35
        { 1, 1, { 1, 3}},  // 36
        { 1, 1, { 1,18}},  // 37
        { 0, 1, {18,-2}},  // 38
        { 0, 2, {18,17}},  // 39
        { 0, 1, {-1,18}},  // 40
        { 1, 1, {17, 3}},  // 41
        { 0, 1, {17, 2}},  // 42
        { 0, 1, {17, 3}},  // 43
        { 1, 1, {18, 2}},  // 44
        { 1, 1, {17,-2}},  // 45
        { 0, 1, { 1,-2}},  // 46
        { 0, 2, {18,17}},  // 47
        { 0, 1, {17,-2}},  // 48
        { 1, 1, {17,-2}},  // 49
        { 0, 1, {18, 3}},  // 50
        { 0, 1, { 2,17}},  // 51
        { 1, 2, {18,-3}},  // 52
        { 1, 2, { 1,18}},  // 53
        { 1, 2, {18, 2}},  // 54
        { 0, 1, {17,-1}},  // 55
        { 0, 1, {17,-2}},  // 56
        { 1, 1, {17,-2}},  // 57
        { 1, 1, { 1, 3}},  // 58
        { 0, 1, { 1,17}},  // 59
        { 1, 2, {18,-2}},  // 60
        { 1, 2, {17,-3}},  // 61
        { 0, 2, {18,17}},  // 62
        { 0, 2, {18,17}},  // 63
};

static const WavpackDecorrSpec default_specs [] = {
        { 1, 2, {18,18, 2,17, 3}},         // 0
        { 0, 2, {18,17,-1, 3, 2}},         // 1
        { 1, 1, {17,18,18,-2, 2}},         // 2
        { 0, 2, {18,17, 3,-2,17}},         // 3
        { 1, 2, {18,17, 2,17, 3}},         // 4
        { 0, 1, {18,18,-1, 2,17}},         // 5
        { 0, 1, {17,17,-2, 2, 3}},         // 6
        { 0, 1, {18,-2,18, 2,17}},         // 7
        { 1, 2, {18,18,-1, 2, 3}},         // 8
        { 0, 2, {18,17, 3, 2, 5}},         // 9
        { 1, 1, {18,17,18, 2, 5}},         // 10
        { 0, 1, {17,17,-2, 2, 3}},         // 11
        { 0, 1, {18,-2,18, 2, 5}},         // 12
        { 0, 1, {17,-2,17, 2,-3}},         // 13
        { 1, 1, {17,-2,17, 1, 2}},         // 14
        { 0, 1, {17,17,-2, 2, 3}},         // 15
        { 1, 1, {18, 3, 1, 5, 4}},         // 16
        { 1, 4, {18,18, 2, 3,-2}},         // 17
        { 0, 1, { 1,-1,-1, 2,17}},         // 18
        { 0, 2, {18,17, 3, 2, 5}},         // 19
        { 0, 1, {18,18,18, 2,17}},         // 20
        { 0, 1, {18,17,-1, 2,18}},         // 21
        { 1, 1, {17, 3, 2, 1, 7}},         // 22
        { 0, 2, {18,-2,18, 2, 3}},         // 23
        { 1, 3, {18,-3,18, 2, 3}},         // 24
        { 0, 3, {18,17, 2, 3,17}},         // 25
        { 1, 1, {17,17, 2, 1, 4}},         // 26
        { 0, 1, {17,18,-2, 2,17}},         // 27
        { 1, 1, {18,18, 3, 5, 2}},         // 28
        { 0, 1, {17,17, 2,18, 4}},         // 29
        { 0, 1, {18,17, 1, 4, 6}},         // 30
        { 1, 1, { 3,17,18, 2,17}},         // 31
        { 1, 1, {17, 3, 2, 1, 7}},         // 32
        { 0, 1, {18,17,-1, 2, 3}},         // 33
        { 1, 1, {17,17, 2, 1, 4}},         // 34
        { 1, 2, {18,17,-1,17, 3}},         // 35
        { 1, 2, {18,17, 2, 3,-1}},         // 36
        { 0, 2, {18,18,-2, 2,17}},         // 37
        { 0, 1, {17,17, 2,18, 4}},         // 38
        { 0, 5, {-2,18,18,18, 2}},         // 39
        { 1, 1, {18,18,-1, 6, 3}},         // 40
        { 0, 1, {17,17,-2, 2, 3}},         // 41
        { 1, 1, {18,17,18, 2,17}},         // 42
        { 0, 1, {18,17, 4, 3, 1}},         // 43
        { 0, 1, {-2,18, 2, 2,18}},         // 44
        { 1, 2, {18,18,-2, 2,-1}},         // 45
        { 1, 1, {17,17, 2, 1, 4}},         // 46
        { 0, 1, {17,18,-2, 2,17}},         // 47
        { 1, 1, {17, 3, 2, 1, 7}},         // 48
        { 1, 3, {18,-3,18, 2, 3}},         // 49
        { 1, 2, {18,18,-2, 2,-1}},         // 50
        { 1, 1, {18,18, 3, 5, 2}},         // 51
        { 0, 2, {18,18,-1, 2,17}},         // 52
        { 0, 1, {18,-1,17,18, 2}},         // 53
        { 0, 1, {17,-1, 2, 3, 6}},         // 54
        { 0, 1, {18,-2,18, 2, 5}},         // 55
        { 1, 2, {18,18,-2, 2,-1}},         // 56
        { 0, 3, {18,18, 2, 3,17}},         // 57
        { 0, 1, {17,17, 2,18, 4}},         // 58
        { 1, 1, {17,-2,17, 1, 2}},         // 59
        { 0, 1, {-1, 3, 5, 4, 7}},         // 60
        { 0, 3, {18,18, 3, 2, 5}},         // 61
        { 0, 1, {17,17, 2,18, 4}},         // 62
        { 0, 1, {18,17,-2,18, 3}},         // 63
};

static const WavpackDecorrSpec high_specs [] = {
        { 1, 2, {18,18,18,-2, 2, 3, 5,-1,17, 4}},  // 0
        { 0, 1, {18,17,-2, 2,18, 3, 7, 2, 5, 4}},  // 1
        { 1, 2, { 1,18, 3, 6,-2,18, 2, 3, 4, 5}},  // 2
        { 0, 2, {18,18,-2, 2,18, 3, 6, 2,17, 4}},  // 3
        { 1, 2, {18,18, 2,18, 3, 2,-1, 4,18, 5}},  // 4
        { 1, 1, { 7, 6, 5, 3, 4, 2, 5, 4, 3, 7}},  // 5
        { 1, 1, {17, 3,18, 7, 2, 6, 1, 4, 3, 5}},  // 6
        { 1, 1, {-2,18,18,18, 3,-2, 6, 5, 2, 1}},  // 7
        { 1, 2, {18,18,-1,18, 2, 3, 6,-2,17, 5}},  // 8
        { 0, 1, {17,17,18, 3, 6, 4, 5, 2,18,-2}},  // 9
        { 1, 2, { 1,18,-2, 3, 5, 2, 4,-1, 6, 1}},  // 10
        { 0, 2, {18,18, 3, 6,18, 2, 4, 8, 5, 3}},  // 11
        { 0, 1, {-2, 1,18, 2,-2, 7,18, 2,-1, 5}},  // 12
        { 1, 1, { 4, 3, 8, 1, 5, 2, 5, 6, 2, 8}},  // 13
        { 1, 1, {17,18, 2, 6, 3, 4,-1, 1, 8, 6}},  // 14
        { 0, 1, {18,18, 3, 6, 3,-2, 2, 5,-1, 1}},  // 15
        { 0, 1, {18,18,17,-1, 2,-2,18, 3, 4, 5}},  // 16
        { 1, 2, {18,17, 2,-2,18, 3, 5, 7, 2, 4}},  // 17
        { 1, 2, {18,18, 3, 6,-2,18, 2, 5, 8, 3}},  // 18
        { 0, 1, {18,17, 2,18,18, 2, 6, 5,17, 7}},  // 19
        { 1, 2, {18,17, 2,18, 3, 2, 6,18,-1, 4}},  // 20
        { 1, 1, { 5, 3, 6, 5, 3, 4, 1, 2, 4, 7}},  // 21
        { 1, 1, { 5, 3, 6, 5, 3, 4, 1, 2, 4, 7}},  // 22
        { 0, 1, {-2,18,18,18,-2, 3, 2, 4, 6, 5}},  // 23
        { 1, 2, {18,17,-3, 3,-1,18, 2, 3, 6, 5}},  // 24
        { 0, 1, {17,18, 7, 3,-2, 7, 1, 2, 4, 5}},  // 25
        { 1, 1, { 2,18,18,-2, 2, 4,-1,18, 3, 6}},  // 26
        { 0, 3, { 1,18, 4, 3, 5, 2, 4,18, 2, 3}},  // 27
        { 0, 1, {-2,18, 2,18, 3, 7,18, 2, 6,-2}},  // 28
        { 0, 1, {-1,18,18, 2,18, 3, 5,18, 2, 8}},  // 29
        { 1, 1, {18,18, 5, 4, 6, 4, 5, 1, 4, 3}},  // 30
        { 1, 1, {18, 3, 6, 5, 7, 8, 2, 3, 1,-1}},  // 31
        { 1, 1, {18,18,18, 2,-2, 3, 5,18, 2, 8}},  // 32
        { 0, 2, {18,17,-2, 2, 3,18,-3, 5, 2, 7}},  // 33
        { 1, 1, { 1, 1,-1, 8,17, 3,-2, 2, 6,17}},  // 34
        { 0, 2, {18,18,17, 2,-2, 3, 2, 4,18, 5}},  // 35
        { 1, 1, {17,18, 2,-1, 5, 7,18, 3, 4, 6}},  // 36
        { 1, 1, { 5, 4, 5,17, 3, 6, 3, 4, 7, 2}},  // 37
        { 0, 1, {17, 3, 1, 7, 4, 2, 5,-2,18, 6}},  // 38
        { 0, 1, {17,18, 2,18, 4, 3, 5, 7,-3, 6}},  // 39
        { 1, 2, {17,17,-3,-2, 2, 8,18,-1, 3, 5}},  // 40
        { 0, 1, {17,17,18, 2, 3, 6,-2, 8, 1, 7}},  // 41
        { 1, 1, { 1, 2, 6,-2,18, 2, 5,-3, 7,-2}},  // 42
        { 0, 1, {18,18, 3,18, 6, 8,-2, 2, 3, 5}},  // 43
        { 0, 1, {18,17, 2,18,-2, 3, 7, 6, 2, 4}},  // 44
        { 0, 1, {-1,18,18, 2,18, 3, 5,18, 2, 8}},  // 45
        { 1, 1, {18,18, 2,-1, 3, 6, 1, 3, 4, 8}},  // 46
        { 0, 1, {18,18, 3, 6, 5, 3,-2, 2,18,-1}},  // 47
        { 0, 1, {18,17,-3,18, 2, 4,-2, 3, 6,17}},  // 48
        { 1, 3, { 1, 2,17, 3,18, 7,-1, 5, 2, 4}},  // 49
        { 1, 1, {18, 3,18, 6, 8,18,-2, 5, 7, 2}},  // 50
        { 0, 1, {17, 2,18, 6, 3, 2, 5, 4, 8, 1}},  // 51
        { 0, 1, {18,17,-1, 2, 3,18,18, 2, 3,17}},  // 52
        { 1, 1, {18, 7, 6, 5, 5, 3, 1, 4, 2, 4}},  // 53
        { 1, 1, { 6,17, 3, 8, 1, 5, 7,-1, 2, 1}},  // 54
        { 1, 1, {18,-2,18, 3,-2, 2, 7, 4, 6,18}},  // 55
        { 1, 3, {18,-3,18, 2, 3,18,-1, 7, 2, 5}},  // 56
        { 0, 2, {18,-2, 7, 1, 3, 2, 4, 6,-3, 7}},  // 57
        { 1, 1, {18,-2, 2,-3,18,-2,17,-1, 4, 2}},  // 58
        { 0, 3, {17,17, 2, 5, 3, 7,18, 6, 4, 2}},  // 59
        { 0, 1, {-1,18,18, 2,18, 3, 5,18, 2, 8}},  // 60
        { 0, 1, {-1,18,18, 2,18, 3, 5,18, 2, 8}},  // 61
        { 1, 1, {18,17, 4, 6, 6, 4, 5, 3, 4, 1}},  // 62
        { 0, 1, {18, 5, 3, 6, 2, 3, 8, 1, 3, 7}},  // 63
};

static const WavpackDecorrSpec very_high_specs [] = {
        { 1, 2, {18,18, 2, 3,-2,18, 2, 4, 7, 5, 3, 6, 8,-1,18, 2}},        // 0
        { 0, 1, {18,18,-1,18, 2, 3, 4, 6, 5, 7,18,-3, 8, 2,-1, 3}},        // 1
        { 1, 2, { 1,18,-2, 4,18, 2, 3, 6,-1, 7, 5,-2,18, 8, 2, 4}},        // 2
        { 0, 1, {17,17, 2, 3, 4,18,-1, 5, 6, 7,18, 2, 8,17, 3,-2}},        // 3
        { 1, 1, {18,18, 2,18, 3, 2,18, 4,-1, 3,18, 2, 6, 8,17, 5}},        // 4
        { 0, 2, {18,17, 2, 3,-2, 5,18,-3, 2, 4, 7, 3, 6, 8, 5,17}},        // 5
        { 1, 1, {18,-2, 2,-3,18, 5,-2,18, 2, 3, 6, 2,17, 4, 7,-1}},        // 6
        { 1, 1, {17, 8,18, 3,-2, 2, 5, 4,18, 6, 3, 8, 7, 2, 5, 4}},        // 7
        { 0, 2, {18,17,-2, 2,18, 3, 2, 5,-3, 4, 7,18, 3, 8, 6, 2}},        // 8
        { 1, 1, { 3, 6, 5, 5, 1, 3, 7, 4, 2, 6, 4,18, 3, 7, 5, 6}},        // 9
        { 1, 2, { 1,18, 3, 2,-2, 1, 5, 4, 6, 2, 7, 1, 8, 3,-1, 1}},        // 10
        { 0, 1, {18,18, 2, 3, 6, 3, 5,-2, 2, 4,18, 3,-2,-1, 6, 7}},        // 11
        { 0, 1, {-2,18, 2,18, 7, 2, 6,-2, 3, 4,18,18, 2,-3, 8, 5}},        // 12
        { 0, 2, {18,18,18, 2, 4, 3,18, 5, 3, 6,-2, 2, 4,18, 8, 7}},        // 13
        { 0, 1, {-2, 1,18, 2,-2,18,-1, 5, 7, 2, 3, 4,18, 2, 6, 2}},        // 14
        { 1, 1, {17,18, 3, 2, 1, 7,-1, 2, 4, 3, 5, 6,-2,18, 7, 8}},        // 15
        { 1, 1, {18,18, 2,18, 3, 4, 6,-2,18, 5, 8, 2, 3, 7, 4,-1}},        // 16
        { 0, 1, {18,18,18,-1, 2, 3, 4, 6, 8,18, 3, 5, 2, 6, 7, 4}},        // 17
        { 1, 1, {17,-2,18,18, 2, 5, 3, 8, 2,-1, 6, 1, 3, 4, 7, 5}},        // 18
        { 0, 1, {17,17,18, 2, 3, 6,-2, 8, 1, 7, 5, 2, 3, 1, 4, 8}},        // 19
        { 1, 1, {17,17, 3, 2, 7, 1, 4, 3, 6, 2, 5,-2, 8, 7,18, 6}},        // 20
        { 0, 1, {18,17,-2, 2,18, 3,-3, 7, 6, 5, 2, 4,-1, 8, 3,17}},        // 21
        { 1, 1, { 2,18,18,-2, 2, 4,-1, 5,18, 3, 8, 6, 2, 7,17, 4}},        // 22
        { 0, 1, {17, 3, 6, 8, 5, 4, 3, 8, 1,18, 7, 2, 4, 5, 6, 3}},        // 23
        { 1, 2, {17,18, 4, 8, 3, 2, 5, 7, 6, 8, 2, 7,-2,18, 3, 4}},        // 24
        { 1, 1, { 6, 5, 5, 3, 4, 7, 3, 2, 4, 6, 3, 7, 1, 5, 2, 4}},        // 25
        { 1, 1, { 1,18,-1, 2, 1, 3, 8,-2, 2, 5, 6, 3, 8, 7,18, 4}},        // 26
        { 0, 1, { 1,17,-1,18, 3, 2, 5, 4, 6, 7, 8, 3, 4, 2, 1,-2}},        // 27
        { 0, 1, {18, 2,18,18, 2,18, 6,-2,18, 7, 5, 4, 3, 2,18,-2}},        // 28
        { 0, 3, { 1, 4,18, 3, 2, 4, 1, 5, 2, 3, 6,18, 8, 7, 2, 4}},        // 29
        { 0, 1, {17,-2, 1,-3, 2,18, 3,-2, 4,18, 3, 6, 7,-3, 2, 8}},        // 30
        { 1, 1, {17,18,18, 4, 2, 3, 7, 6,18, 8, 5,-1, 4, 2, 3,17}},        // 31
        { 1, 2, {18,-1,17,18, 2, 3,-2,18, 5, 8, 2, 4, 3, 7, 6,-1}},        // 32
        { 1, 1, {18,18,18,-2, 4, 2, 3,18, 5, 8, 2, 4, 6, 7,-2, 3}},        // 33
        { 1, 2, {18,18,-2,18,-1, 3, 2, 5,18,-2, 7, 2, 3, 4, 6, 8}},        // 34
        { 0, 1, {17,18,-1, 2, 4,18, 8, 3, 6, 5, 7,-3, 2, 4, 3,17}},        // 35
        { 1, 1, {18,18,17, 2,-1,18, 3, 2,18, 6, 5, 4,18, 7, 2,-1}},        // 36
        { 0, 2, { 1,18,-1,18, 3, 2, 4, 6,-3, 7,-1, 5, 1, 2, 3, 8}},        // 37
        { 1, 1, { 1,17,-2, 2,-3, 6, 3, 5, 1, 2, 7, 6, 8,-2, 4, 1}},        // 38
        { 0, 1, {17,-1, 5, 1, 4, 3, 6, 2,-2,18, 3, 2, 4, 5, 8,-1}},        // 39
        { 0, 2, {18,18,17, 2, 3,-2, 5,18, 2, 4, 7, 8, 6,17, 3, 5}},        // 40
        { 1, 1, { 1, 5, 1, 3, 4, 3, 7, 5, 1, 3, 6, 1, 2, 4, 3, 8}},        // 41
        { 1, 2, { 1,-1, 3, 2,18, 7,-2, 5, 2, 6, 4, 3,-1,18, 8, 7}},        // 42
        { 0, 2, {18,17, 3,18, 2, 5, 4, 3, 6, 2, 7, 8,18, 3, 4, 5}},        // 43
        { 1, 1, { 3, 6,17, 8, 7, 5,18,-1, 1, 2, 3, 4, 2, 6, 8, 1}},        // 44
        { 0, 2, {18,18, 3,-3,18, 2, 6, 5, 3, 7,18, 4,-2, 8, 2, 3}},        // 45
        { 0, 1, {-1,18,18, 2,18, 3, 5,18, 2,18, 6, 8, 4, 5, 7,-1}},        // 46
        { 1, 1, {17, 1, 7, 2, 3,18,-2, 3, 6, 4, 2, 7, 8, 5, 3,17}},        // 47
        { 1, 1, { 3, 6, 5, 5, 1, 3, 7, 4, 2, 6, 4,18, 3, 7, 5, 6}},        // 48
        { 0, 1, {18,18,18, 2, 4,-1,18, 8,-1, 2, 3, 4, 6,-2, 1, 7}},        // 49
        { 1, 1, {18,-2,17,18, 2, 6, 3,-2, 5, 4, 7, 1,-3, 8, 2, 6}},        // 50
        { 0, 1, {17,18,18, 4, 2, 7, 3, 6,-2,18, 8, 4, 5, 2, 7,17}},        // 51
        { 1, 1, {18,18, 5, 4, 6, 4, 1, 5, 4, 3, 2, 5, 6, 1, 4, 5}},        // 52
        { 0, 1, {18,18,-2,18, 2,-3, 3, 8, 5,18, 6, 4, 3,-1, 7, 2}},        // 53
        { 1, 1, {18, 2,-2,-3,18, 5, 2, 3,-2, 4, 6, 1,-3, 2, 7, 8}},        // 54
        { 0, 1, {18, 3, 5, 8, 2, 6, 7, 3, 1, 5, 2,-1, 8, 6, 7, 4}},        // 55
        { 1, 1, { 4, 3, 8, 1, 5, 6, 2, 5, 8,-2, 2, 7, 3,18, 5, 4}},        // 56
        { 0, 1, {-1,18,18, 2,18, 3, 5,18, 2,18, 6, 8, 4, 5, 7,-1}},        // 57
        { 1, 1, {17, 3,18,18, 7, 2, 4,18, 6, 2, 3,-1, 8, 5,18,-3}},        // 58
        { 0, 1, { 3,17,18, 2,18, 6, 7,-3,18, 2, 5, 6, 3, 8, 7,-1}},        // 59
        { 1, 1, {18,18, 2,18,18, 2,-1, 7, 3,18, 5, 2, 6, 4,-1,18}},        // 60
        { 0, 3, {18, 3, 4, 1, 5, 2,18, 4, 2, 3,18, 7, 6, 1, 2, 4}},        // 61
        { 0, 1, {-1,18,18, 2,18, 3, 5,18, 2,18, 6, 8, 4, 5, 7,-1}},        // 62
        { 1, 1, {17, 1,18, 2, 3, 6, 4, 5, 7,18, 3, 8, 2, 4,-2,17}},        // 63
};

const WavpackDecorrSpec *get_fast_specs (void) { return fast_specs; }
int get_num_fast_specs (void) { return sizeof (fast_specs) / sizeof (fast_specs [0]); }

const WavpackDecorrSpec *get_default_specs (void) { return default_specs; }
int get_num_default_specs (void) { return sizeof (default_specs) / sizeof (default_specs [0]); }

const WavpackDecorrSpec *get_high_specs (void) { return high_specs; }
int get_num_high_specs (void) { return sizeof (high_specs) / sizeof (high_specs [0]); }

const WavpackDecorrSpec *get_very_high_specs (void) { return very_high_specs; }
int get_num_very_high_specs (void) { return sizeof (very_high_specs) / sizeof (very_high_specs [0]); }


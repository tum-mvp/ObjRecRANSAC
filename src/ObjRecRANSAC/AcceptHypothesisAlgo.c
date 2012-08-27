# 1 "/tmp/tmpxft_000002c9_00000000-1_AcceptHypothesisAlgo.cudafe1.gpu"
# 19 "SVD.h"
struct __T10;
# 16 "SVD.h"
union __T11;
# 12 "SVD.h"
struct Vector3;
# 52 "SVD.h"
struct __T12;
# 49 "SVD.h"
union __T13;
# 46 "SVD.h"
struct Matrix3x3;
# 211 "SVD.h"
struct __T14;
# 208 "SVD.h"
union __T15;
# 205 "SVD.h"
struct Quaternion;
# 62 "AcceptHypothesisAlgo.h"
struct FloatType_2;
# 66 "AcceptHypothesisAlgo.h"
struct RangeImage;
# 214 "/usr/lib/gcc/i686-apple-darwin10/4.2.1/include/stddef.h" 3
typedef unsigned long size_t;
#include "crt/host_runtime.h"
# 115 "/usr/include/i386/_types.h" 3
typedef unsigned long __darwin_clock_t;
# 77 "/usr/include/time.h" 3
typedef __darwin_clock_t clock_t;
# 5 "SVD.h"
typedef float FloatType;
# 19 "SVD.h"
struct __T10 {
# 20 "SVD.h"
FloatType x;
# 20 "SVD.h"
FloatType y;
# 20 "SVD.h"
FloatType z;};
# 16 "SVD.h"
union __T11 {
# 17 "SVD.h"
FloatType v[3];
# 19 "SVD.h"
struct  {
# 20 "SVD.h"
FloatType x;
# 20 "SVD.h"
FloatType y;
# 20 "SVD.h"
FloatType z;};};
# 12 "SVD.h"
struct Vector3 {
# 16 "SVD.h"
union  {
# 17 "SVD.h"
FloatType v[3];
# 19 "SVD.h"
struct  {
# 20 "SVD.h"
FloatType x;
# 20 "SVD.h"
FloatType y;
# 20 "SVD.h"
FloatType z;};};};
# 52 "SVD.h"
struct __T12 {
# 53 "SVD.h"
FloatType m00;
# 53 "SVD.h"
FloatType m01;
# 53 "SVD.h"
FloatType m02;
# 54 "SVD.h"
FloatType m10;
# 54 "SVD.h"
FloatType m11;
# 54 "SVD.h"
FloatType m12;
# 55 "SVD.h"
FloatType m20;
# 55 "SVD.h"
FloatType m21;
# 55 "SVD.h"
FloatType m22;};
# 49 "SVD.h"
union __T13 {
# 50 "SVD.h"
FloatType v[9];
# 52 "SVD.h"
struct  {
# 53 "SVD.h"
FloatType m00;
# 53 "SVD.h"
FloatType m01;
# 53 "SVD.h"
FloatType m02;
# 54 "SVD.h"
FloatType m10;
# 54 "SVD.h"
FloatType m11;
# 54 "SVD.h"
FloatType m12;
# 55 "SVD.h"
FloatType m20;
# 55 "SVD.h"
FloatType m21;
# 55 "SVD.h"
FloatType m22;};};
# 46 "SVD.h"
struct Matrix3x3 {
# 49 "SVD.h"
union  {
# 50 "SVD.h"
FloatType v[9];
# 52 "SVD.h"
struct  {
# 53 "SVD.h"
FloatType m00;
# 53 "SVD.h"
FloatType m01;
# 53 "SVD.h"
FloatType m02;
# 54 "SVD.h"
FloatType m10;
# 54 "SVD.h"
FloatType m11;
# 54 "SVD.h"
FloatType m12;
# 55 "SVD.h"
FloatType m20;
# 55 "SVD.h"
FloatType m21;
# 55 "SVD.h"
FloatType m22;};};};
# 211 "SVD.h"
struct __T14 {
# 212 "SVD.h"
FloatType x;
# 212 "SVD.h"
FloatType y;
# 212 "SVD.h"
FloatType z;
# 212 "SVD.h"
FloatType w;};
# 208 "SVD.h"
union __T15 {
# 209 "SVD.h"
FloatType v[4];
# 211 "SVD.h"
struct  {
# 212 "SVD.h"
FloatType x;
# 212 "SVD.h"
FloatType y;
# 212 "SVD.h"
FloatType z;
# 212 "SVD.h"
FloatType w;};};
# 205 "SVD.h"
struct Quaternion {
# 208 "SVD.h"
union  {
# 209 "SVD.h"
FloatType v[4];
# 211 "SVD.h"
struct  {
# 212 "SVD.h"
FloatType x;
# 212 "SVD.h"
FloatType y;
# 212 "SVD.h"
FloatType z;
# 212 "SVD.h"
FloatType w;};};};
# 62 "AcceptHypothesisAlgo.h"
struct FloatType_2 {
# 63 "AcceptHypothesisAlgo.h"
FloatType x;
# 63 "AcceptHypothesisAlgo.h"
FloatType y;};
# 66 "AcceptHypothesisAlgo.h"
struct RangeImage {
# 69 "AcceptHypothesisAlgo.h"
int width;
# 69 "AcceptHypothesisAlgo.h"
int height;
# 70 "AcceptHypothesisAlgo.h"
FloatType mInvPixelSize;
# 71 "AcceptHypothesisAlgo.h"
FloatType mBounds[4];
# 73 "AcceptHypothesisAlgo.h"
FloatType *mGridSetsP;
# 74 "AcceptHypothesisAlgo.h"
struct FloatType_2 *mPixels;};
void *memcpy(void*, const void*, size_t); void *memset(void*, int, size_t);
# 120 "AcceptHypothesisAlgo.h"
__asm(".align 2");

#include "AcceptHypothesisAlgo.stub.c"

; NOTE: Assertions have been autogenerated by utils/update_analyze_test_checks.py
; RUN: opt < %s -mtriple=riscv64 -mattr=+v,+zfh,+experimental-zvfh -riscv-v-vector-bits-min=256 -passes='print<cost-model>' -cost-kind=throughput 2>&1 -disable-output | FileCheck %s --check-prefix=FP-REDUCE

define void @reduce_fadd_half() {
; FP-REDUCE-LABEL: 'reduce_fadd_half'
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 2 for instruction: %V1 = call fast half @llvm.vector.reduce.fadd.v1f16(half 0xH0000, <1 x half> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 3 for instruction: %V2 = call fast half @llvm.vector.reduce.fadd.v2f16(half 0xH0000, <2 x half> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 4 for instruction: %V4 = call fast half @llvm.vector.reduce.fadd.v4f16(half 0xH0000, <4 x half> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 5 for instruction: %V8 = call fast half @llvm.vector.reduce.fadd.v8f16(half 0xH0000, <8 x half> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 6 for instruction: %V16 = call fast half @llvm.vector.reduce.fadd.v16f16(half 0xH0000, <16 x half> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 7 for instruction: %v32 = call fast half @llvm.vector.reduce.fadd.v32f16(half 0xH0000, <32 x half> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 8 for instruction: %V64 = call fast half @llvm.vector.reduce.fadd.v64f16(half 0xH0000, <64 x half> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 9 for instruction: %V128 = call fast half @llvm.vector.reduce.fadd.v128f16(half 0xH0000, <128 x half> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 1 for instruction: ret void
;
  %V1 = call fast half @llvm.vector.reduce.fadd.v1f16(half 0.0, <1 x half> undef)
  %V2 = call fast half @llvm.vector.reduce.fadd.v2f16(half 0.0, <2 x half> undef)
  %V4 = call fast half @llvm.vector.reduce.fadd.v4f16(half 0.0, <4 x half> undef)
  %V8 = call fast half @llvm.vector.reduce.fadd.v8f16(half 0.0, <8 x half> undef)
  %V16 = call fast half @llvm.vector.reduce.fadd.v16f16(half 0.0, <16 x half> undef)
  %v32 = call fast half @llvm.vector.reduce.fadd.v32f16(half 0.0, <32 x half> undef)
  %V64 = call fast half @llvm.vector.reduce.fadd.v64f16(half 0.0, <64 x half> undef)
  %V128 = call fast half @llvm.vector.reduce.fadd.v128f16(half 0.0, <128 x half> undef)
  ret void
}

define void @reduce_fadd_float() {
; FP-REDUCE-LABEL: 'reduce_fadd_float'
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 2 for instruction: %V1 = call fast float @llvm.vector.reduce.fadd.v1f32(float 0.000000e+00, <1 x float> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 3 for instruction: %V2 = call fast float @llvm.vector.reduce.fadd.v2f32(float 0.000000e+00, <2 x float> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 4 for instruction: %V4 = call fast float @llvm.vector.reduce.fadd.v4f32(float 0.000000e+00, <4 x float> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 5 for instruction: %V8 = call fast float @llvm.vector.reduce.fadd.v8f32(float 0.000000e+00, <8 x float> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 6 for instruction: %V16 = call fast float @llvm.vector.reduce.fadd.v16f32(float 0.000000e+00, <16 x float> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 7 for instruction: %v32 = call fast float @llvm.vector.reduce.fadd.v32f32(float 0.000000e+00, <32 x float> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 8 for instruction: %V64 = call fast float @llvm.vector.reduce.fadd.v64f32(float 0.000000e+00, <64 x float> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 10 for instruction: %V128 = call fast float @llvm.vector.reduce.fadd.v128f32(float 0.000000e+00, <128 x float> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 1 for instruction: ret void
;
  %V1 = call fast float @llvm.vector.reduce.fadd.v1f32(float 0.0, <1 x float> undef)
  %V2 = call fast float @llvm.vector.reduce.fadd.v2f32(float 0.0, <2 x float> undef)
  %V4 = call fast float @llvm.vector.reduce.fadd.v4f32(float 0.0, <4 x float> undef)
  %V8 = call fast float @llvm.vector.reduce.fadd.v8f32(float 0.0, <8 x float> undef)
  %V16 = call fast float @llvm.vector.reduce.fadd.v16f32(float 0.0, <16 x float> undef)
  %v32 = call fast float @llvm.vector.reduce.fadd.v32f32(float 0.0, <32 x float> undef)
  %V64 = call fast float @llvm.vector.reduce.fadd.v64f32(float 0.0, <64 x float> undef)
  %V128 = call fast float @llvm.vector.reduce.fadd.v128f32(float 0.0, <128 x float> undef)
  ret void
}

define void @reduce_fadd_double() {
; FP-REDUCE-LABEL: 'reduce_fadd_double'
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 2 for instruction: %V1 = call fast double @llvm.vector.reduce.fadd.v1f64(double 0.000000e+00, <1 x double> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 3 for instruction: %V2 = call fast double @llvm.vector.reduce.fadd.v2f64(double 0.000000e+00, <2 x double> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 4 for instruction: %V4 = call fast double @llvm.vector.reduce.fadd.v4f64(double 0.000000e+00, <4 x double> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 5 for instruction: %V8 = call fast double @llvm.vector.reduce.fadd.v8f64(double 0.000000e+00, <8 x double> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 6 for instruction: %V16 = call fast double @llvm.vector.reduce.fadd.v16f64(double 0.000000e+00, <16 x double> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 7 for instruction: %v32 = call fast double @llvm.vector.reduce.fadd.v32f64(double 0.000000e+00, <32 x double> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 9 for instruction: %V64 = call fast double @llvm.vector.reduce.fadd.v64f64(double 0.000000e+00, <64 x double> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 12 for instruction: %V128 = call fast double @llvm.vector.reduce.fadd.v128f64(double 0.000000e+00, <128 x double> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 1 for instruction: ret void
;
  %V1 = call fast double @llvm.vector.reduce.fadd.v1f64(double 0.0, <1 x double> undef)
  %V2 = call fast double @llvm.vector.reduce.fadd.v2f64(double 0.0, <2 x double> undef)
  %V4 = call fast double @llvm.vector.reduce.fadd.v4f64(double 0.0, <4 x double> undef)
  %V8 = call fast double @llvm.vector.reduce.fadd.v8f64(double 0.0, <8 x double> undef)
  %V16 = call fast double @llvm.vector.reduce.fadd.v16f64(double 0.0, <16 x double> undef)
  %v32 = call fast double @llvm.vector.reduce.fadd.v32f64(double 0.0, <32 x double> undef)
  %V64 = call fast double @llvm.vector.reduce.fadd.v64f64(double 0.0, <64 x double> undef)
  %V128 = call fast double @llvm.vector.reduce.fadd.v128f64(double 0.0, <128 x double> undef)
  ret void
}

define void @reduce_oredered_fadd_half() {
; FP-REDUCE-LABEL: 'reduce_oredered_fadd_half'
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 3 for instruction: %V1 = call half @llvm.vector.reduce.fadd.v1f16(half 0xH0000, <1 x half> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 4 for instruction: %V2 = call half @llvm.vector.reduce.fadd.v2f16(half 0xH0000, <2 x half> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 6 for instruction: %V4 = call half @llvm.vector.reduce.fadd.v4f16(half 0xH0000, <4 x half> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 10 for instruction: %V8 = call half @llvm.vector.reduce.fadd.v8f16(half 0xH0000, <8 x half> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 18 for instruction: %V16 = call half @llvm.vector.reduce.fadd.v16f16(half 0xH0000, <16 x half> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 34 for instruction: %v32 = call half @llvm.vector.reduce.fadd.v32f16(half 0xH0000, <32 x half> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 66 for instruction: %V64 = call half @llvm.vector.reduce.fadd.v64f16(half 0xH0000, <64 x half> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 130 for instruction: %V128 = call half @llvm.vector.reduce.fadd.v128f16(half 0xH0000, <128 x half> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 1 for instruction: ret void
;
  %V1 = call half @llvm.vector.reduce.fadd.v1f16(half 0.0, <1 x half> undef)
  %V2 = call half @llvm.vector.reduce.fadd.v2f16(half 0.0, <2 x half> undef)
  %V4 = call half @llvm.vector.reduce.fadd.v4f16(half 0.0, <4 x half> undef)
  %V8 = call half @llvm.vector.reduce.fadd.v8f16(half 0.0, <8 x half> undef)
  %V16 = call half @llvm.vector.reduce.fadd.v16f16(half 0.0, <16 x half> undef)
  %v32 = call half @llvm.vector.reduce.fadd.v32f16(half 0.0, <32 x half> undef)
  %V64 = call half @llvm.vector.reduce.fadd.v64f16(half 0.0, <64 x half> undef)
  %V128 = call half @llvm.vector.reduce.fadd.v128f16(half 0.0, <128 x half> undef)
  ret void
}

define void @reduce_oredered_fadd_float() {
; FP-REDUCE-LABEL: 'reduce_oredered_fadd_float'
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 3 for instruction: %V1 = call float @llvm.vector.reduce.fadd.v1f32(float 0.000000e+00, <1 x float> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 4 for instruction: %V2 = call float @llvm.vector.reduce.fadd.v2f32(float 0.000000e+00, <2 x float> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 6 for instruction: %V4 = call float @llvm.vector.reduce.fadd.v4f32(float 0.000000e+00, <4 x float> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 10 for instruction: %V8 = call float @llvm.vector.reduce.fadd.v8f32(float 0.000000e+00, <8 x float> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 18 for instruction: %V16 = call float @llvm.vector.reduce.fadd.v16f32(float 0.000000e+00, <16 x float> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 34 for instruction: %v32 = call float @llvm.vector.reduce.fadd.v32f32(float 0.000000e+00, <32 x float> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 66 for instruction: %V64 = call float @llvm.vector.reduce.fadd.v64f32(float 0.000000e+00, <64 x float> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 131 for instruction: %V128 = call float @llvm.vector.reduce.fadd.v128f32(float 0.000000e+00, <128 x float> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 1 for instruction: ret void
;
  %V1 = call float @llvm.vector.reduce.fadd.v1f32(float 0.0, <1 x float> undef)
  %V2 = call float @llvm.vector.reduce.fadd.v2f32(float 0.0, <2 x float> undef)
  %V4 = call float @llvm.vector.reduce.fadd.v4f32(float 0.0, <4 x float> undef)
  %V8 = call float @llvm.vector.reduce.fadd.v8f32(float 0.0, <8 x float> undef)
  %V16 = call float @llvm.vector.reduce.fadd.v16f32(float 0.0, <16 x float> undef)
  %v32 = call float @llvm.vector.reduce.fadd.v32f32(float 0.0, <32 x float> undef)
  %V64 = call float @llvm.vector.reduce.fadd.v64f32(float 0.0, <64 x float> undef)
  %V128 = call float @llvm.vector.reduce.fadd.v128f32(float 0.0, <128 x float> undef)
  ret void
}

define void @reduce_oredered_fadd_double() {
; FP-REDUCE-LABEL: 'reduce_oredered_fadd_double'
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 3 for instruction: %V1 = call double @llvm.vector.reduce.fadd.v1f64(double 0.000000e+00, <1 x double> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 4 for instruction: %V2 = call double @llvm.vector.reduce.fadd.v2f64(double 0.000000e+00, <2 x double> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 6 for instruction: %V4 = call double @llvm.vector.reduce.fadd.v4f64(double 0.000000e+00, <4 x double> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 10 for instruction: %V8 = call double @llvm.vector.reduce.fadd.v8f64(double 0.000000e+00, <8 x double> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 18 for instruction: %V16 = call double @llvm.vector.reduce.fadd.v16f64(double 0.000000e+00, <16 x double> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 34 for instruction: %v32 = call double @llvm.vector.reduce.fadd.v32f64(double 0.000000e+00, <32 x double> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 67 for instruction: %V64 = call double @llvm.vector.reduce.fadd.v64f64(double 0.000000e+00, <64 x double> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 133 for instruction: %V128 = call double @llvm.vector.reduce.fadd.v128f64(double 0.000000e+00, <128 x double> undef)
; FP-REDUCE-NEXT:  Cost Model: Found an estimated cost of 1 for instruction: ret void
;
  %V1 = call double @llvm.vector.reduce.fadd.v1f64(double 0.0, <1 x double> undef)
  %V2 = call double @llvm.vector.reduce.fadd.v2f64(double 0.0, <2 x double> undef)
  %V4 = call double @llvm.vector.reduce.fadd.v4f64(double 0.0, <4 x double> undef)
  %V8 = call double @llvm.vector.reduce.fadd.v8f64(double 0.0, <8 x double> undef)
  %V16 = call double @llvm.vector.reduce.fadd.v16f64(double 0.0, <16 x double> undef)
  %v32 = call double @llvm.vector.reduce.fadd.v32f64(double 0.0, <32 x double> undef)
  %V64 = call double @llvm.vector.reduce.fadd.v64f64(double 0.0, <64 x double> undef)
  %V128 = call double @llvm.vector.reduce.fadd.v128f64(double 0.0, <128 x double> undef)
  ret void
}

declare half @llvm.vector.reduce.fadd.v1f16(half, <1 x half>)
declare half @llvm.vector.reduce.fadd.v2f16(half, <2 x half>)
declare half @llvm.vector.reduce.fadd.v4f16(half, <4 x half>)
declare half @llvm.vector.reduce.fadd.v8f16(half, <8 x half>)
declare half @llvm.vector.reduce.fadd.v16f16(half, <16 x half>)
declare half @llvm.vector.reduce.fadd.v32f16(half, <32 x half>)
declare half @llvm.vector.reduce.fadd.v64f16(half, <64 x half>)
declare half @llvm.vector.reduce.fadd.v128f16(half, <128 x half>)
declare float @llvm.vector.reduce.fadd.v1f32(float, <1 x float>)
declare float @llvm.vector.reduce.fadd.v2f32(float, <2 x float>)
declare float @llvm.vector.reduce.fadd.v4f32(float, <4 x float>)
declare float @llvm.vector.reduce.fadd.v8f32(float, <8 x float>)
declare float @llvm.vector.reduce.fadd.v16f32(float, <16 x float>)
declare float @llvm.vector.reduce.fadd.v32f32(float, <32 x float>)
declare float @llvm.vector.reduce.fadd.v64f32(float, <64 x float>)
declare float @llvm.vector.reduce.fadd.v128f32(float, <128 x float>)
declare double @llvm.vector.reduce.fadd.v1f64(double, <1 x double>)
declare double @llvm.vector.reduce.fadd.v2f64(double, <2 x double>)
declare double @llvm.vector.reduce.fadd.v4f64(double, <4 x double>)
declare double @llvm.vector.reduce.fadd.v8f64(double, <8 x double>)
declare double @llvm.vector.reduce.fadd.v16f64(double, <16 x double>)
declare double @llvm.vector.reduce.fadd.v32f64(double, <32 x double>)
declare double @llvm.vector.reduce.fadd.v64f64(double, <64 x double>)
declare double @llvm.vector.reduce.fadd.v128f64(double, <128 x double>)

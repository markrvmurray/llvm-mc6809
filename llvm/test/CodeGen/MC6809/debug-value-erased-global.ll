; RUN: llc -mtriple=mc6809 -stop-after=mc6809-internalize %s -o - | FileCheck %s
; RUN: llc -mtriple=mc6809 -filetype=obj %s -o /dev/null

; Only a debug value keeps @unused alive by the time libcalls are internalized,
; and metadata is not a use, so the pass erases it. The machine debug value must
; not go on naming the erased global, since emitting its location follows the
; pointer. The udiv is here to make the pass run: it emits a contingent libcall.

; CHECK-LABEL: name: f
; CHECK-NOT: @unused
; CHECK: DBG_VALUE $noreg, $noreg, !8
; CHECK-NOT: undef

@unused = internal constant [4 x i8] c"abc\00", !dbg !9

define i16 @f(i16 %a, i16 %b) !dbg !4 {
  call void @llvm.dbg.value(metadata ptr @unused, metadata !8, metadata !DIExpression()), !dbg !12
  %r = udiv i16 %a, %b, !dbg !12
  ret i16 %r, !dbg !12
}

define i16 @__udivhi3(i16 %a, i16 %b) partition "contingent" {
  ret i16 0
}

declare void @llvm.dbg.value(metadata, metadata, metadata)

!llvm.dbg.cu = !{!0}
!llvm.module.flags = !{!2, !3}

!0 = distinct !DICompileUnit(language: DW_LANG_C11, file: !1, emissionKind: FullDebug)
!1 = !DIFile(filename: "t.c", directory: "/")
!2 = !{i32 2, !"Debug Info Version", i32 3}
!3 = !{i32 7, !"Dwarf Version", i32 4}
!4 = distinct !DISubprogram(name: "f", scope: !1, file: !1, type: !5, spFlags: DISPFlagDefinition, unit: !0, retainedNodes: !7)
!5 = !DISubroutineType(types: !6)
!6 = !{null}
!7 = !{!8}
!8 = !DILocalVariable(name: "v", scope: !4, file: !1, type: !10)
!9 = !DIGlobalVariableExpression(var: !13, expr: !DIExpression())
!10 = !DIDerivedType(tag: DW_TAG_pointer_type, baseType: !11, size: 16)
!11 = !DIBasicType(name: "char", size: 8, encoding: DW_ATE_signed_char)
!12 = !DILocation(line: 1, scope: !4)
!13 = distinct !DIGlobalVariable(name: "unused", scope: !0, file: !1, type: !11, isDefinition: true)

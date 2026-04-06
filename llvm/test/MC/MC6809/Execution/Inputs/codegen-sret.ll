; LLVM IR for struct return (sret) execution test.
%struct.Pair = type { i16, i16 }

define dso_local void @make_pair(ptr sret(%struct.Pair) %result, i16 noundef %x, i16 noundef %y) local_unnamed_addr {
  %p1 = getelementptr inbounds %struct.Pair, ptr %result, i32 0, i32 0
  store i16 %x, ptr %p1
  %p2 = getelementptr inbounds %struct.Pair, ptr %result, i32 0, i32 1
  store i16 %y, ptr %p2
  ret void
}

//===-- MC6809Toolchain.cpp - MC6809 ToolChain ----------------------------===//
//
// Part of the LLVM Project, under Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MC6809Toolchain.h"

#include "clang/Driver/CommonArgs.h"

#include "clang/Driver/Compilation.h"
#include "clang/Driver/Driver.h"
#include "clang/Options/Options.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/Path.h"

using namespace llvm::opt;
using namespace clang::driver;
using namespace clang::driver::tools;
using namespace clang::driver::toolchains;

MC6809ToolChain::MC6809ToolChain(const Driver &D, const llvm::Triple &Triple,
         const llvm::opt::ArgList &Args)
    : ToolChain(D, Triple, Args) {
  getProgramPaths().push_back(getDriver().Dir);
}

Tool *MC6809ToolChain::buildLinker() const { return new tools::mc6809::Linker(*this); }

// The C library that goes with this clang, found the way the rest of the
// toolchain is found: beside the binary that was invoked.  `--sysroot` wins,
// which is what a developer pointing at a live picolibc build directory uses;
// otherwise it is <clang>/../lib/clang-runtimes/<triple>, the layout the
// upstream bare-metal toolchains use, so a moved or renamed installation
// still works and nothing has to be baked in when the compiler is built.
std::string MC6809ToolChain::computeSysRoot() const {
  const Driver &D = getDriver();
  if (!D.SysRoot.empty())
    return D.SysRoot;
  llvm::SmallString<128> Dir(D.Dir);
  llvm::sys::path::append(Dir, "..", "lib", "clang-runtimes",
                          getTriple().str());
  return std::string(Dir);
}

// Only offer the sysroot if there is a library in it: without one, a link
// gets the same arguments it got before, so a tree that has never had a
// picolibc installed beside it behaves exactly as it did.
static bool sysRootHasLibC(llvm::StringRef SysRoot) {
  llvm::SmallString<128> LibC(SysRoot);
  llvm::sys::path::append(LibC, "lib", "libc.a");
  return llvm::sys::fs::exists(LibC);
}

void MC6809ToolChain::AddClangSystemIncludeArgs(const ArgList &DriverArgs,
                                    ArgStringList &CC1Args) const {
  if (DriverArgs.hasArg(options::OPT_nostdinc))
    return;

  if (!DriverArgs.hasArg(options::OPT_nobuiltininc)) {
    SmallString<128> Dir(getDriver().ResourceDir);
    llvm::sys::path::append(Dir, "include");
    addSystemInclude(DriverArgs, CC1Args, Dir.str());
    // The resourcedir's `include/` already holds os9.h (staged by the
    // mc6809-os9-runtime target), so `#include <os9.h>` resolves from
    // there without another path.
  }

  if (DriverArgs.hasArg(options::OPT_nostdlibinc))
    return;

  // The C library's own headers, when one is installed beside this clang.
  std::string SysRoot = computeSysRoot();
  if (sysRootHasLibC(SysRoot)) {
    SmallString<128> Inc(SysRoot);
    llvm::sys::path::append(Inc, "include");
    addSystemInclude(DriverArgs, CC1Args, Inc.str());
  }
}

void MC6809ToolChain::addClangTargetOptions(const ArgList &DriverArgs,
                                ArgStringList &CC1Args, BoundArch,
                                Action::OffloadKind) const {
  CC1Args.push_back("-nostdsysteminc");
  // Not yet implemented for GlobalISel.
  CC1Args.push_back("-fexperimental-assignment-tracking=disabled");

  // Bug #163 Phase 2: OS-9 / NitrOS-9 syscalls and library functions
  // traditionally use `$` in identifiers (F$Exit, I$Read, I$Write, …).
  // The MC6809 assembler tokenises a `$` inside an identifier
  // context as part of the name (only a leading `$` is the hex
  // prefix), and gcc6809 has had `-fdollars-in-identifiers` enabled
  // for OS-9 builds forever.  Match that posture for the OS-9 triple
  // so ports of existing NitrOS-9 code work without per-file flags.
  //
  // Pushed BEFORE the user's own args end up on the cc1 line, so a
  // user-supplied `-fno-dollars-in-identifiers` still wins via
  // last-occurrence override.
  if (getTriple().isOSOS9())
    CC1Args.push_back("-fdollars-in-identifiers");
}

static bool hasLTOEmitAsm(const ArgList &Args) {
  for (Arg *A : Args) {
    if (!A->getOption().matches(clang::options::OPT_Wl_COMMA) &&
        !A->getOption().matches(clang::options::OPT_Xlinker))
      continue;
    if (A->containsValue("--lto-emit-asm"))
      return true;
  }
  return false;
}

void mc6809::Linker::ConstructJob(Compilation &C, const JobAction &JA,
                               const InputInfo &Output,
                               const InputInfoList &Inputs, const ArgList &Args,
                               const char *LinkingOutput) const {
  ArgStringList CmdArgs;

  auto &TC = static_cast<const toolchains::MC6809ToolChain &>(getToolChain());
  auto &D = TC.getDriver();

  // A partial link is not a final link: there is no entry point to keep
  // anything alive, so collecting unused sections would collect all of them,
  // and sorting by alignment means nothing without a layout to sort.  Left
  // in, `-r` produces an object holding a file name and nothing else -- which
  // is what every start-up object picolibc installed used to be.
  const bool IsRelocatable = Args.hasArg(options::OPT_r);

  // Pass defaults before AddLinkerInputs, since that includes -Wl
  // options, which should override these.
  if (!IsRelocatable) {
    CmdArgs.push_back("--gc-sections");
    CmdArgs.push_back("--sort-section=alignment");
  }

  AddLinkerInputs(TC, Inputs, Args, CmdArgs, JA);

  AddLTOOptions(TC, Args, Output, Inputs, CmdArgs);

  if (!D.SysRoot.empty())
    CmdArgs.push_back(Args.MakeArgString("--sysroot=" + D.SysRoot));

  // Bug #161 Phase 1.5: when targeting HD6309, prepend the multilib
  // subdirectory holding the HD6309-optimised compiler-rt builtins.
  // compiler-rt builds two variants of libclang_rt.builtins.a; the
  // HD6309 copy is staged at <resourcedir>/lib/<triple>/hd6309/. By
  // prepending -L for that subdir BEFORE the default library paths,
  // any subsequent `-lclang_rt.builtins` (including the one picolibc
  // emits inside its --start-group block) resolves to the HD6309
  // version first. The fallback library is unchanged for plain 6809
  // builds.
  {
    StringRef CPU = Args.getLastArgValue(options::OPT_mcpu_EQ);
    if (CPU == "hd6309" || CPU == "6309") {
      SmallString<128> HD6309LibPath(D.ResourceDir);
      llvm::sys::path::append(HD6309LibPath, "lib",
                              TC.getTriple().getTriple(), "hd6309");
      CmdArgs.push_back(
          Args.MakeArgString(Twine("-L") + HD6309LibPath.str()));
    }
  }

  TC.AddFilePathLibArgs(Args, CmdArgs);
  Args.addAllArgs(CmdArgs, {options::OPT_L, options::OPT_T_Group,
                            options::OPT_e, options::OPT_s, options::OPT_t,
                            options::OPT_Z_Flag, options::OPT_r});

  // Bug #163 Phase 2: when the OS in the triple is OS-9 (NitrOS-9),
  // diverge from the bare-metal default link line:
  //   - skip the bare-metal startup libs (crt0, crt, c, mc6809rt) —
  //     the OS-9 CRT lives in compiler-rt/lib/builtins/mc6809-os9/
  //     and the picolibc-on-OS9 port (Phase 4) will provide an
  //     OS-9-aware libc.  At Phase 2 PoC the user passes objects
  //     directly (crt0.o + syscalls.o + their main).
  //   - use mc6809-os9.lds (flat OUTPUT_FORMAT(binary) body) instead
  //     of link.ld (which assumes a flash/RAM split irrelevant to OS-9
  //     module loading).
  //   - the OS-9 module body still needs the os9-link post-link
  //     wrapper to prepend the header + append CRC; the driver does
  //     NOT auto-invoke os9-link yet — that lands with the end-to-end
  //     `hello.c` commit, which threads __mem_size out of the link
  //     into the os9-link --mem argument.
  const bool IsOS9  = TC.getTriple().isOSOS9();
  const bool IsDECB = TC.getTriple().isOSDECB();

  // A C library installed beside the compiler.  Bare metal used to name a
  // runtime that is not shipped -- link.ld, libcrt0.a and libcrt.a exist
  // nowhere -- so with a sysroot it gets picolibc's shape instead: its
  // start-up object, its library and system layer, and its linker script.
  // Without one, nothing changes, and a build that passes its own -L and -T
  // is unaffected.
  const std::string SysRoot = TC.computeSysRoot();
  const bool HaveSysRoot = !IsOS9 && sysRootHasLibC(SysRoot);
  if (HaveSysRoot && !Args.hasArg(options::OPT_nostdlib)) {
    llvm::SmallString<128> LibDir(SysRoot);
    llvm::sys::path::append(LibDir, "lib");
    CmdArgs.push_back(Args.MakeArgString(Twine("-L") + LibDir.str()));
  }

  // DECB: the entry and the linker script are staged in the per-triple
  // directory, the way OS-9's are.  There is no library yet, so that is all
  // a link gets -- a program brings its own code.
  if (IsDECB && !Args.hasArg(options::OPT_nostartfiles, options::OPT_nostdlib))
    CmdArgs.push_back("-l:crt0.o");

  // DECB's library, when one is installed beside the compiler: libdecb is the
  // console and the handful of calls a C library needs to exist -- there is no
  // operating system here beyond a character in and a character out.
  if (IsDECB && HaveSysRoot &&
      !Args.hasArg(options::OPT_nodefaultlibs, options::OPT_nostdlib)) {
    CmdArgs.push_back("--start-group");
    if (!Args.hasArg(options::OPT_nolibc))
      CmdArgs.push_back("-lc");
    CmdArgs.push_back("-ldecb");
    CmdArgs.push_back("-lclang_rt.builtins");
    CmdArgs.push_back("--end-group");
  }

  if (!IsOS9 && !IsDECB &&
      !Args.hasArg(options::OPT_nostartfiles, options::OPT_nostdlib)) {
    // Prefixing a colon causes GNU LD-like linkers to search for this filename
    // as-is. This contains the minimum necessary startup library.
    //
    // picolibc installs several: the plain one does not call exit(), because
    // a program on hardware does not return from main -- under a simulator
    // that means it prints its output and then runs for ever.  -mcrt0= picks
    // another; `semihost` is the one a simulator wants.
    StringRef Crt0 = Args.getLastArgValue(options::OPT_mcrt0_EQ);
    if (Crt0.empty() || Crt0 == "default")
      CmdArgs.push_back("-l:crt0.o");
    else
      CmdArgs.push_back(Args.MakeArgString("-l:crt0-" + Crt0 + ".o"));

    // libcrt0.a contains optional startup objects that are only pulled in if
    // referenced.  It belongs to the older runtime, not to a sysroot.
    if (!HaveSysRoot)
      CmdArgs.push_back("-lcrt0");
  } else if (!IsOS9 && !IsDECB && !IsRelocatable &&
             !Args.hasArg(options::OPT_e)) {
    // No crt0 means no _start. Set a dummy entry point to suppress
    // the "cannot find entry symbol" linker warning.  Not for a partial
    // link, which has no entry to set.
    CmdArgs.push_back("-e0");
  }

  if (!IsOS9 && !IsDECB && !HaveSysRoot &&
      !Args.hasArg(options::OPT_nodefaultlibs, options::OPT_nostdlib))
    CmdArgs.push_back("-lcrt");

  if (!IsOS9 && !IsDECB && !HaveSysRoot &&
      !Args.hasArg(options::OPT_nodefaultlibs, options::OPT_nolibc,
                   options::OPT_nostdlib))
    CmdArgs.push_back("-lc");

  // With a sysroot: the library and the layer that gives it a machine to run
  // on, as a group -- each needs the other -- and the compiler's own
  // builtins, which the older runtime spelled libmc6809rt.
  if (HaveSysRoot && !IsDECB &&
      !Args.hasArg(options::OPT_nodefaultlibs, options::OPT_nostdlib)) {
    CmdArgs.push_back("--start-group");
    if (!Args.hasArg(options::OPT_nolibc))
      CmdArgs.push_back("-lc");
    CmdArgs.push_back("-lsemihost");
    CmdArgs.push_back("-lclang_rt.builtins");
    CmdArgs.push_back("--end-group");
  }

  // MC6809 runtime library: hand-written assembly builtins for shift, multiply,
  // divide operations that the hardware can't do natively.
  if (!IsOS9 && !IsDECB && !HaveSysRoot &&
      !Args.hasArg(options::OPT_nodefaultlibs, options::OPT_nostdlib))
    CmdArgs.push_back("-lmc6809rt");

  // Default linker script.  Bare-metal uses link.ld; OS-9 uses
  // mc6809-os9.lds (the script landed alongside the OS-9 CRT).
  // -nostdlib or an explicit -T suppresses both.
  if (!Args.hasArg(options::OPT_T, options::OPT_nostdlib)) {
    if (IsOS9)
      CmdArgs.push_back("-Tmc6809-os9.lds");
    else if (IsDECB)
      CmdArgs.push_back("-Tmc6809-decb.lds");
    else if (HaveSysRoot)
      CmdArgs.push_back("-Tpicolibc.ld");
    else
      CmdArgs.push_back("-Tlink.ld");
  }

  // For OS-9, prepend the resourcedir's per-triple library directory
  // so lld finds mc6809-os9.lds AND libclang_rt.os9.a.  Then pull in
  // libclang_rt.os9 (forces crt0.o + syscalls.o into the link) unless
  // the user opted out via -nostartfiles/-nostdlib.
  if (IsOS9) {
    SmallString<128> OS9LibPath(D.ResourceDir);
    llvm::sys::path::append(OS9LibPath, "lib", "mc6809-unknown-os9");
    CmdArgs.push_back(Args.MakeArgString(Twine("-L") + OS9LibPath.str()));

    if (!Args.hasArg(options::OPT_nostartfiles, options::OPT_nostdlib))
      CmdArgs.push_back("-lclang_rt.os9");

    // The C library, when one is installed beside this clang.  libos9 is
    // its system layer -- the OS-9 calls behind read, write, open and the
    // rest -- and each needs the other: the library calls the layer, and the
    // layer's errno is the library's.  A group settles that without either
    // having to come first.  Without a sysroot the link is unchanged, which
    // is what a build against a picolibc build directory relies on.
    std::string SysRoot = getToolChain().computeSysRoot();
    if (!Args.hasArg(options::OPT_nodefaultlibs, options::OPT_nostdlib) &&
        sysRootHasLibC(SysRoot)) {
      SmallString<128> LibDir(SysRoot);
      llvm::sys::path::append(LibDir, "lib");
      CmdArgs.push_back(Args.MakeArgString(Twine("-L") + LibDir.str()));
      CmdArgs.push_back("--start-group");
      CmdArgs.push_back("-lc");
      CmdArgs.push_back("-los9");
      CmdArgs.push_back("--end-group");
    }

    // The OS-9 build of the compiler builtins (integer only, from the same
    // per-triple resource directory).
    if (!Args.hasArg(options::OPT_nodefaultlibs, options::OPT_nostdlib))
      CmdArgs.push_back("-lclang_rt.builtins");

    // Bug #163 Phase 3: derive --os9-name from the output file's stem
    // (basename minus extension) unless the user already passed it.
    // mc6809-os9.lds defaults OUTPUT_FORMAT to os9-program-module, so
    // the linker will demand a name and emit a wrapped module.
    bool UserGaveName = false;
    for (const char *A : CmdArgs) {
      StringRef SA(A);
      if (SA.starts_with("--os9-name") ||
          SA.starts_with("-Wl,--os9-name")) {
        UserGaveName = true;
        break;
      }
    }
    if (!UserGaveName) {
      StringRef Stem = llvm::sys::path::stem(Output.getFilename());
      if (!Stem.empty())
        CmdArgs.push_back(
            Args.MakeArgString(Twine("--os9-name=") + Stem));
    }

    // HD6309-native modules use Prgrm|Obj6309 ($17); plain 6809 uses
    // Prgrm|Objct ($11).  --os9-type defaults to $11 in lld, so we
    // only need to override for HD6309.
    StringRef CPU = Args.getLastArgValue(options::OPT_mcpu_EQ);
    if (CPU == "hd6309" || CPU == "6309")
      CmdArgs.push_back("--os9-type=0x17");
  }

  // For DECB, the resourcedir's per-triple directory holds the start-up code
  // and the linker script; the compiler's builtins are the bare-metal ones,
  // the machine being the same, so that directory is on the path too rather
  // than a second copy of an identical library.
  if (IsDECB) {
    SmallString<128> DECBLibPath(D.ResourceDir);
    llvm::sys::path::append(DECBLibPath, "lib", "mc6809-unknown-decb");
    CmdArgs.push_back(Args.MakeArgString(Twine("-L") + DECBLibPath.str()));
    SmallString<128> BuiltinsPath(D.ResourceDir);
    llvm::sys::path::append(BuiltinsPath, "lib", "mc6809-unknown-unknown");
    CmdArgs.push_back(Args.MakeArgString(Twine("-L") + BuiltinsPath.str()));
  }

  CmdArgs.push_back("-o");
  CmdArgs.push_back(Output.getFilename());

  C.addCommand(std::make_unique<Command>(JA, *this, ResponseFileSupport::None(),
                                         Args.MakeArgString(TC.GetLinkerPath()),
                                         CmdArgs, Inputs, Output));
  if (!hasLTOEmitAsm(Args)) {
    for (StringRef PostLinkTool :
         Args.getAllArgValues(options::OPT_fpost_link_tool)) {
      ArgStringList PostLinkToolArgs;
      PostLinkToolArgs.push_back(
          Args.MakeArgString(Twine(Output.getFilename()) + ".elf"));

      std::string Path = PostLinkTool.str();
      if (!llvm::sys::fs::exists(Path))
        Path = getToolChain().GetProgramPath(Path.c_str());

      C.addCommand(std::make_unique<Command>(
          JA, *this, ResponseFileSupport::None(),
          Args.MakeArgString(Path.c_str()), PostLinkToolArgs, Inputs, Output));
    }
  }
}

void mc6809::Linker::AddLTOOptions(const toolchains::MC6809ToolChain &TC, const ArgList &Args,
                                const InputInfo &Output,
                                const InputInfoList &Inputs,
                                ArgStringList &CmdArgs) const {
  assert(!Inputs.empty() && "Must have at least one input.");
  addLTOOptions(TC, Args, CmdArgs, Output, Inputs,
                TC.getLTOMode(Args) == LTOK_Thin);
  addMC6809CodeGenArgs(CmdArgs);
  unsigned DPBytes = 0;
  StringRef LTODP = Args.getLastArgValue(options::OPT_mlto_dp_EQ);
  if (!LTODP.empty()) {
    if (LTODP.getAsInteger(0, DPBytes))
      TC.getDriver().Diag(diag::err_drv_invalid_dp) << LTODP;
  }
  for (const std::string &Val :
       Args.getAllArgValues(options::OPT_mreserve_dp_EQ)) {
    if (Val.empty())
      continue;
    unsigned Amt;
    if (StringRef(Val).getAsInteger(0, Amt)) {
      TC.getDriver().Diag(diag::err_drv_invalid_dp) << Val;
      continue;
    }
    if (DPBytes < Amt) {
      TC.getDriver().Diag(diag::err_drv_too_much_dp_reserved) << Val;
      continue;
    }
    DPBytes -= Amt;
  }
  if (DPBytes) {
    CmdArgs.push_back("-mllvm");
    CmdArgs.push_back(Args.MakeArgString("-dp-avail=" + Twine(DPBytes)));
  }
}

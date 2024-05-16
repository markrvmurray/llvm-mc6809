#!/usr/bin/env python3

import sys
import copy
import ctypes

from instructions import RawInstructions
from indexmodes import IndexModes

def defs(reg):
	if reg in "AB":
		return ["AD", "AQ"]
	if reg == "D":
		return ["AA", "AB", "AQ"]
	if reg in "EF":
		return ["AW", "AQ"]
	if reg == "W":
		return ["AE", "AF", "AQ"]
	if reg == "Q":
		return ["AA", "AB", "AD", "AE", "AF", "AW"]
	if reg in "XY":
		return ["I" + reg]
	if reg in "SU":
		return ["S" + reg]
	raise Exception("OINQUE! Illegal register value '{}'".format(reg))

class Generate:
	def __init__(self, filename, marker_start="// MRVM START MARKER", marker_finish="// MRVM END MARKER"):
		self.filename = filename
		self.marker_start = marker_start
		self.marker_finish = marker_finish
		self.contents = []

	def __enter__(self):
		self.filehandle = open(self.filename, "r")
		for line in self.filehandle.readlines():
			self.contents.append(line.rstrip())
		self.filehandle.close()
		self.filehandle = open(self.filename, "w")
		for line in self.contents:
			self.filehandle.write(line + '\n')
			if line == self.marker_start:
				break
		return self.filehandle

	def __exit__(self, *args):
		preamble_done = False
		for line in self.contents:
			if line == self.marker_finish:
				preamble_done = True
			if preamble_done:
				self.filehandle.write(line + '\n')
		self.filehandle.close()

Instructions = [ [], [], [] ]
InstrFormat = { }
AddressMode = { }
MultiClass = { }
DecoderTables = set()
ModeEnums = set()

def format_inst(params):
	instr1 = []
	instr23 = []
	for param in params:
		if param[0][1] == -1:
			instr1 += [ "let P1Inst{{{}}} = {};".format(param[0][0], param[1]) ]
			instr23 += [ "let P23Inst{{{}}} = {};".format(param[0][0] + 8, param[1]) ]
		else:
			instr1 += [ "let P1Inst{{{}-{}}} = {};".format(param[0][0], param[0][1], param[1]) ]
			instr23 += [ "let P23Inst{{{}-{}}} = {};".format(param[0][0] + 8, param[0][1] + 8, param[1]) ]
	instr1 = ' '.join(instr1)
	instr23 = ' '.join(instr23 + [ "let P23Inst{7-0} = Pre{7-0};" ])
	return [ instr1, instr23 ]

def insert_instruction(instruction):
	instruction["class"] = 'MC6809{addressmode}_P{page}'.format(**instruction)
	if instruction["size"] > 0:
		if instruction["page"] != 1:
			instruction["operandsize"] = instruction["size"] - 2
		else:
			instruction["operandsize"] = instruction["size"] - 1
		instruction["inst"] = format_inst(instruction["params"])
	instruction["sizebits"] = instruction["size"]*8
	if len(instruction["lets"]["Constraints"]) > 0:
		instruction["lets"]["Constraints"] = '"{}"'.format(", ".join(instruction["lets"]["Constraints"]))
	else:
		del instruction["lets"]["Constraints"]
	if len(instruction["lets"]["Predicates"]) > 0:
		instruction["lets"]["Predicates"] = "[" + ", ".join(instruction["lets"]["Predicates"]) + "]"
	else:
		del instruction["lets"]["Predicates"]
	instruction["inslist"] = ", ".join(instruction["ins"]) if len(instruction["ins"]) else ""
	instruction["useslist"] = ",".join(instruction["uses"]) if len(instruction["uses"]) else ""
	instruction["outsformatted"] = "(outs {})".format(", ".join(instruction["outs"])) if len(instruction["outs"]) else "(outs)"
	instruction["outslist"] = ", ".join(instruction["outs"])
	instruction["defslist"] = ",".join(instruction["defs"])
	instruction["useslist"] = ",".join(instruction["uses"])
	# The main event
	Instructions[instruction["page"] - 1].append(instruction)
	if instruction["size"] > 0:
		DecoderTables.add(("DecoderTable_Page_{page}_Size_{sizebits}".format(**instruction), instruction["size"]))
		ModeEnums.add(instruction["mode"]+instruction["indexmode"])
		if instruction["operand"] != "":
			instruction["operand"] = " " + instruction["operand"]
		InstrFormat[instruction["class"]] = instruction
		AddressMode[instruction["addressmode"]] = { "addressmode":instruction["addressmode"],
							    "mode":instruction["mode"],
							    "indexmode":instruction["indexmode"],
							    "page":instruction["page"],
							    "operand":instruction["operand"],
							    "inst":instruction["inst"],
							    "operandsize":instruction["operandsize"],
							    "insformatted":"(ins {})".format(", ".join(instruction["ins"]))
									    if len(instruction["ins"]) else "(ins)" }

def insert_indexmode(indexmode):
	indexmode["class"] = 'MC6809{addressmode}_P{page}'.format(**indexmode)
	if indexmode["page"] != 1:
		indexmode["size"] = indexmode["operandsize"] + 2
	else:
		indexmode["size"] = indexmode["operandsize"] + 1
	indexmode["sizebits"] = indexmode["size"]*8
	indexmode["inslist"] = ", ".join(indexmode["ins"]) if len(indexmode["ins"]) else ""
	indexmode["useslist"] = ",".join(indexmode["uses"]) if len(indexmode["uses"]) else ""
	indexmode["outsformatted"] = "(outs {})".format(", ".join(indexmode["outs"])) if len(indexmode["outs"]) else "(outs)"
	indexmode["outslist"] = ", ".join(indexmode["outs"])
	indexmode["defslist"] = ",".join(indexmode["defs"])
	indexmode["useslist"] = ",".join(indexmode["uses"])
	indexmode["inst"] = format_inst(indexmode["params"])
	if indexmode["lets"]["Constraints"]:
		indexmode["lets"]["Constraints"] = '"{}"'.format(", ".join(indexmode["lets"]["Constraints"]))
	else:
		del indexmode["lets"]["Constraints"]
	if indexmode["lets"]["Predicates"]:
		indexmode["lets"]["Predicates"] = "[" + ", ".join(indexmode["lets"]["Predicates"]) + "]"
	else:
		del indexmode["lets"]["Predicates"]
	if len(indexmode["lets"]) > 0:
		indexmode["letsformatted"] = " {\n    " + "\n    ".join([ "let " + k + " = " + indexmode["lets"][k] + ";" for k in sorted(indexmode["lets"]) ]) + "\n  }"
	else:
		indexmode["letsformatted"] = ";"
	# The main event
	MultiClass[indexmode["multiclass"]][indexmode["indexnumber"]] = \
			'  def {indexmode} : {class}<!con(outdag, {outsformatted}), Mnemonic, opcode, !listconcat(defsin, [{defslist}]), !listconcat(usesin, [{useslist}])>{letsformatted}'.format(**indexmode)
	DecoderTables.add(("DecoderTable_Page_{page}_Size_{sizebits}".format(**indexmode), indexmode["size"]))
	ModeEnums.add(indexmode["mode"]+indexmode["indexmode"])
	InstrFormat[indexmode["class"]] = indexmode
	indexmode["operand"] = " " + indexmode["operand"]
	AddressMode[indexmode["addressmode"]] = { "addressmode":indexmode["addressmode"],
						  "mode":indexmode["mode"],
						  "indexmode":indexmode["indexmode"],
						  "page":indexmode["page"],
						  "operand":indexmode["operand"],
						  "operandsize":indexmode["operandsize"],
						  "inst":indexmode["inst"],
						  "insformatted":"(ins {})".format(", ".join(indexmode["ins"]))
								  if len(indexmode["ins"]) else "(ins)" }

for instr_ in RawInstructions:
	instr = copy.deepcopy(instr_)
	truncmode = instr["mode"]
	if truncmode[0] == "1":
		truncmode = truncmode[1]
	instr["lcmnemonic"] = instr["mnemonic"].lower()
	instr["name"] = instr["mnemonic"] + truncmode
	if not instr.get("defs", None):
		instr["defs"] = []
	if not instr.get("uses", None):
		instr["uses"] = []
	if instr.get("lets", None) is None:
		instr["lets"] = {}
	if instr["lets"].get("Constraints", None) is None:
		instr["lets"]["Constraints"] = []
	if instr["lets"].get("Predicates", None) is None:
		instr["lets"]["Predicates"] = []
	if instr["hd6309"]:
		instr["lets"]["Predicates"] += ["IsHD6309"]

	# The available functions/modes are:
	# "function":"a", Arithmetic - ADD, SUB, XOR, etc
	#  "mode":"1a",    Accumulator Implicit
	#  "mode":"d",     Direct
	#  "mode":"e",     Extended
	#  "mode":"i",     Indexed
	#  "mode":"i8",    Immediate 8-bit
	#  "mode":"i16",   Immediate 16-bit
	#  "mode":"id",    Immediate Direct
	#  "mode":"ie",    Immediate Extended
	#  "mode":"ii",    Immediate Indirect
	#  "mode":"p",     2-Register Postbyte
	# "function":"b", Bit instructions like BIAND
	#  "mode":"bd",   Bit Direct
	# "function":"c", Compares and tests.
	#  "mode":"1a",   Single operand Accumulator or Memory TST
	#  "mode":"a",    Accumulator
	#  "mode":"d",    Direct
	#  "mode":"e",    Extended
	#  "mode":"i",    Indexed
	#  "mode":"i8",   Immediate 8-bit
	#  "mode":"i16",  Immediate 16-bit
	#  "mode":"p",    2-Register Postbyte
	# "function":"j", JMP and JSR
	#  "mode":"d",   Direct
	#  "mode":"e",   Extended
	#  "mode":"i",   Indexed
	# "function":"l", Loads
	#  "mode":"a",   Accumulator
	#  "mode":"d",   Direct
	#  "mode":"e",   Extended
	#  "mode":"i",   Indexed
	#  "mode":"i8",  Immediate 8-bit
	#  "mode":"i16", Immediate 16-bit
	#  "mode":"i32", Immediate 32-bit
	# "function":"m", EXG, TFR and TFM
	#  "mode":"p",   2-Register Postbyte
	#  "mode":"pp",  2-Register Postbyte block copies
	# "function":"r", Branches
	#  "mode":"b",   Branch
	#  "mode":"bc",  Branch Conditional
	#  "mode":"lb",  LongBranch
	#  "mode":"lbc", LongBranch Conditional
	#  "mode":"r",   Return
	# "function":"s", Stores
	#  "mode":"d",   Direct
	#  "mode":"e",   Extended
	#  "mode":"i",   Indexed
	# "function":"ss", Push and Pull
	#  "mode":"s",   Stack Postbyte
	#  "mode":"a",   Accumulator W Inherent
	# "function":"x", System instructions, NOP, SWI, SYNC, etc
	#  "mode":"i8",  System Immediate
	#  "mode":"x",   System Special

	if instr["function"] == "a":
		if instr["mode"] == "1a":
			if instr["mnemonic"][0:3] in ["ASL", "ASR", "CLR", "COM", "DEC", "INC", "LSR", "NEG", "ROL", "ROR"]:
				mnemonic = instr["mnemonic"][0:3]
				reg = instr["mnemonic"][-1:]
				if reg in "ABDEFW":
					if mnemonic == "ASL":
						instr["uses"] = ["A" + reg]
						instr["defs"] = ["A" + reg, "NZ", "N", "Z", "V", "C"] + defs(reg)
					if mnemonic == "ASR":
						instr["uses"] = ["A" + reg, "C"]
						instr["defs"] = ["A" + reg, "NZ", "N", "Z", "C"] + defs(reg)
					if mnemonic == "CLR":
						instr["defs"] = ["A" + reg, "NZ", "N", "Z", "V", "C"] + defs(reg)
					if mnemonic == "COM":
						instr["uses"] = ["A" + reg]
						instr["defs"] = ["A" + reg, "NZ", "N", "Z", "V", "C"] + defs(reg)
					if mnemonic == "DEC":
						instr["uses"] = ["A" + reg]
						instr["defs"] = ["A" + reg, "NZ", "N", "Z", "V"] + defs(reg)
					if mnemonic == "INC":
						instr["uses"] = ["A" + reg]
						instr["defs"] = ["A" + reg, "NZ", "N", "Z", "V"] + defs(reg)
					if mnemonic == "LSR":
						instr["uses"] = ["A" + reg]
						instr["defs"] = ["A" + reg, "NZ", "N", "Z", "C"] + defs(reg)
					if mnemonic == "NEG":
						instr["uses"] = ["A" + reg]
						instr["defs"] = ["A" + reg, "NZ", "N", "Z", "V", "C"] + defs(reg)
					if mnemonic == "ROL":
						instr["uses"] = ["A" + reg, "C"]
						instr["defs"] = ["A" + reg, "NZ", "N", "Z", "V", "C"] + defs(reg)
					if mnemonic == "ROR":
						instr["uses"] = ["A" + reg, "C"]
						instr["defs"] = ["A" + reg, "NZ", "N", "Z", "C"] + defs(reg)
				else:
					print("Arithmetic sole accumulator register not handled:", instr)
					sys.exit(1)
			else:
				print("Arithmetic accumulator explicit instruction not handled:", instr)
				sys.exit(1)
			instr["mode"] = instr["mode"][-1]
		elif instr["mode"] in ["1d", "1e", "1i"]:
			if instr["mnemonic"] in ["ASL", "ASR", "CLR", "COM", "DEC", "INC", "LSR", "NEG", "ROL", "ROR"]:
				mnemonic = instr["mnemonic"]
				if mnemonic == "ASL":
					instr["defs"] = ["NZ", "N", "Z", "V", "C"]
					instr["lets"]["mayLoad"] = "true"
					instr["lets"]["mayStore"] = "true"
				if mnemonic == "ASR":
					instr["uses"] = ["C"]
					instr["defs"] = ["NZ", "N", "Z", "C"]
					instr["lets"]["mayLoad"] = "true"
					instr["lets"]["mayStore"] = "true"
				if mnemonic == "CLR":
					instr["defs"] = ["NZ", "N", "Z", "V", "C"]
					instr["lets"]["mayStore"] = "true"
				if mnemonic == "COM":
					instr["defs"] = ["NZ", "N", "Z", "V", "C"]
					instr["lets"]["mayLoad"] = "true"
					instr["lets"]["mayStore"] = "true"
				if mnemonic == "DEC":
					instr["defs"] = ["NZ", "N", "Z", "V"]
					instr["lets"]["mayLoad"] = "true"
					instr["lets"]["mayStore"] = "true"
				if mnemonic == "INC":
					instr["defs"] = ["NZ", "N", "Z", "V"]
					instr["lets"]["mayLoad"] = "true"
					instr["lets"]["mayStore"] = "true"
				if mnemonic == "LSR":
					instr["defs"] = ["NZ", "N", "Z", "C"]
					instr["lets"]["mayLoad"] = "true"
					instr["lets"]["mayStore"] = "true"
				if mnemonic == "NEG":
					instr["defs"] = ["NZ", "N", "Z", "V", "C"]
					instr["lets"]["mayLoad"] = "true"
					instr["lets"]["mayStore"] = "true"
				if mnemonic == "ROL":
					instr["uses"] = ["C"]
					instr["defs"] = ["NZ", "N", "Z", "V", "C"]
					instr["lets"]["mayLoad"] = "true"
					instr["lets"]["mayStore"] = "true"
				if mnemonic == "ROR":
					instr["uses"] = ["C"]
					instr["defs"] = ["NZ", "N", "Z", "C"]
					instr["lets"]["mayLoad"] = "true"
					instr["lets"]["mayStore"] = "true"
			else:
				print("Arithmetic RMW not handled:", instr)
				sys.exit(1)
			instr["mode"] = instr["mode"][-1]
		elif instr["mode"] in ["d", "e", "i", "i8", "i16"]:
			if (instr["mnemonic"][0:3] in ["ADD", "ADC", "SUB", "SBC", "AND", "EOR"] or instr["mnemonic"][0:2] in ["OR"]) and instr["mnemonic"][-2:] != "CC":
				mnemonic = instr["mnemonic"][0:3] if instr["mnemonic"][0:2] not in ["OR"] else instr["mnemonic"][0:2]
				reg = instr["mnemonic"][-1:]
				if reg in "ABDEFW":
					if mnemonic == "ADD":
						instr["uses"] += ["A" + reg]
						instr["defs"] += ["A" + reg, "NZ", "N", "Z", "V", "C"] + defs(reg)
						instr["lets"]["mayLoad"] = "true"
						instr["lets"]["isAdd"] = "true"
					if mnemonic == "ADC":
						instr["uses"] += ["A" + reg, "C"]
						instr["defs"] += ["A" + reg, "NZ", "N", "Z", "V", "C"] + defs(reg)
						instr["lets"]["mayLoad"] = "true"
						instr["lets"]["isAdd"] = "true"
					if mnemonic == "SUB":
						instr["uses"] += ["A" + reg]
						instr["defs"] += ["A" + reg, "NZ", "N", "Z", "V", "C"] + defs(reg)
						instr["lets"]["mayLoad"] = "true"
					if mnemonic == "SBC":
						instr["uses"] += ["A" + reg, "C"]
						instr["defs"] += ["A" + reg, "NZ", "N", "Z", "V", "C"] + defs(reg)
						instr["lets"]["mayLoad"] = "true"
					if mnemonic == "AND":
						instr["uses"] += ["A" + reg]
						instr["defs"] += ["A" + reg, "NZ", "N", "Z", "V"] + defs(reg)
						instr["lets"]["mayLoad"] = "true"
					if mnemonic == "OR":
						instr["uses"] += ["A" + reg]
						instr["defs"] += ["A" + reg, "NZ", "N", "Z", "V"] + defs(reg)
						instr["lets"]["mayLoad"] = "true"
					if mnemonic == "EOR":
						instr["uses"] += ["A" + reg]
						instr["defs"] += ["A" + reg, "NZ", "N", "Z", "V"] + defs(reg)
						instr["lets"]["mayLoad"] = "true"
				else:
					print("Arithmetic accumulator register not handled:", instr)
					sys.exit(1)
			elif instr["mnemonic"] in ["DIVD", "DIVQ", "MULD"]:
				if instr["mnemonic"] == "DIVD":
					instr["uses"] = ["AD"]
					instr["defs"] = ["NZ", "N", "Z", "V", "C"] + defs("D")
					instr["lets"]["mayLoad"] = "true"
				if instr["mnemonic"] == "DIVQ":
					instr["uses"] = ["AQ"]
					instr["defs"] = ["NZ", "N", "Z", "V", "C"] + defs("Q")
					instr["lets"]["mayLoad"] = "true"
				if instr["mnemonic"] == "MULD":
					instr["uses"] = ["AD"]
					instr["defs"] = ["AQ", "NZ", "N", "Z"] + defs("Q")
					instr["lets"]["mayLoad"] = "true"
			elif instr["mnemonic"] in ["ANDCC", "ORCC"]:
				instr["uses"] += ["NZ", "N", "Z", "V", "C"]
				instr["defs"] += ["NZ", "N", "Z", "V", "C"]
			else:
				print("Arithmetic accumulator instruction not handled:", instr)
				sys.exit(1)
		elif instr["mode"] in ["id", "ie", "ii"]:
			if instr["mnemonic"] in ["AIM", "OIM", "EIM"]:
				instr["defs"] += ["NZ", "N", "Z", "V"]
				instr["lets"]["mayLoad"] = "true"
				instr["lets"]["mayStore"] = "true"
			else:
				print("Arithmetic immidiate/memory instruction not handled:", instr)
				sys.exit(1)
		elif instr["mode"] in ["p"]:
			if instr["mnemonic"] in ["ADDR", "ADCR", "SUBR", "SBCR", "ANDR", "ORR", "EORR"]:
				mnemonic = instr["mnemonic"]
				if mnemonic == "ADDR":
					instr["defs"] += ["NZ", "N", "Z", "V", "C"]
					instr["ins"] += ["anyregister:$reg1","anyregister:$reg2"]
					instr["outs"] += ["anyregister:$dst"]
					instr["lets"]["isAdd"] = "true"
				if mnemonic == "ADCR":
					instr["uses"] += ["C"]
					instr["defs"] += ["NZ", "N", "Z", "V", "C"]
					instr["ins"] += ["anyregister:$reg1","anyregister:$reg2"]
					instr["outs"] += ["anyregister:$dst"]
					instr["lets"]["isAdd"] = "true"
				if mnemonic == "SUBR":
					instr["defs"] += ["NZ", "N", "Z", "V", "C"]
					instr["ins"] += ["anyregister:$reg1","anyregister:$reg2"]
					instr["outs"] += ["anyregister:$dst"]
				if mnemonic == "SBCR":
					instr["uses"] += ["C"]
					instr["defs"] += ["NZ", "N", "Z", "V", "C"]
					instr["ins"] += ["anyregister:$reg1","anyregister:$reg2"]
					instr["outs"] += ["anyregister:$dst"]
				if mnemonic == "ANDR":
					instr["defs"] += ["NZ", "N", "Z", "V"]
					instr["ins"] += ["anyregister:$reg1","anyregister:$reg2"]
					instr["outs"] += ["anyregister:$dst"]
				if mnemonic == "ORR":
					instr["defs"] += ["NZ", "N", "Z", "V"]
					instr["ins"] += ["anyregister:$reg1","anyregister:$reg2"]
					instr["outs"] += ["anyregister:$dst"]
				if mnemonic == "EORR":
					instr["defs"] += ["NZ", "N", "Z", "V"]
					instr["ins"] += ["anyregister:$reg1","anyregister:$reg2"]
					instr["outs"] += ["anyregister:$dst"]
				instr["lets"]["Constraints"] += ["$dst = $reg2"]
			else:
				print("Arithmetic/Logical postbyte instruction not handled:", instr)
		elif instr["mode"] == "x":
			if instr["mnemonic"] == "DAA":
				instr["uses"] = ["AA"]
				instr["defs"] = ["AA", "NZ", "N", "Z", "C"] + defs("A")
			elif instr["mnemonic"] == "MUL":
				instr["uses"] = ["AA", "AB"]
				instr["defs"] = ["Z", "C"] + defs("D")
			elif instr["mnemonic"] == "SEX":
				instr["uses"] = ["AB"]
				instr["defs"] = ["NZ", "N", "Z"] + defs("D")
			elif instr["mnemonic"] == "SEXW":
				instr["uses"] = ["AD"]
				instr["defs"] = ["NZ", "N", "Z"] + defs("Q")
			else:
				print("Arithmetic inherent instruction not handled:", instr)
				sys.exit(1)
		else:
			print("Arithmetic instruction mode not handled:", instr)
			sys.exit(1)
	elif instr["function"] == "c":
		instr["lets"]["isCompare"] = "true"
		if instr["mode"] == "1a":
			if instr["mnemonic"][0:3] in ["TST"]:
				mnemonic = instr["mnemonic"][0:3]
				reg = instr["mnemonic"][-1:]
				if reg in "ABDEFW":
					instr["uses"] = ["A" + reg]
					instr["defs"] = ["NZ", "N", "Z", "V"]
				else:
					print("Test accumulator register not handled:", instr)
					sys.exit(1)
			else:
				print("Test accumulator instruction not handled:", instr)
				sys.exit(1)
			instr["mode"] = instr["mode"][-1]
		elif instr["mode"] in ["1d", "1e", "1i"]:
			if instr["mnemonic"] in ["TST"]:
				instr["defs"] = ["NZ", "N", "Z", "V"]
			else:
				print("Test memory not handled:", instr)
				sys.exit(1)
			instr["mode"] = instr["mode"][-1]
		elif instr["mode"] in ["d", "e", "i", "i8", "i16"]:
			if instr["mnemonic"][0:3] in ["BIT", "CMP"]:
				mnemonic = instr["mnemonic"][0:3]
				reg = instr["mnemonic"][-1:]
				if reg in "ABDEFWXYSU":
					if reg in "ABDEFW":
						regclass = "A"
					if reg in "XY":
						regclass = "I"
					if reg in "SU":
						regclass = "S"
					if mnemonic == "BIT":
						instr["uses"] += [regclass + reg]
						instr["defs"] += ["NZ", "N", "Z", "V"]
						instr["lets"]["mayLoad"] = "true"
					if mnemonic == "CMP":
						instr["uses"] += [regclass + reg]
						instr["defs"] += ["NZ", "N", "Z", "V", "C"]
						instr["lets"]["mayLoad"] = "true"
				else:
					print("Compare accumulator register not handled:", instr)
					sys.exit(1)
			elif instr["mnemonic"] in ["BITMD"]:
				instr["defs"] += ["Z"]
			else:
				print("Compare accumulator instruction not handled:", instr)
				sys.exit(1)
		elif instr["mode"] in ["id", "ie", "ii"]:
			if instr["mnemonic"] in ["TIM"]:
				instr["defs"] += ["NZ", "N", "Z", "V"]
				instr["lets"]["mayLoad"] = "true"
			else:
				print("Compare immediate/memory instruction not handled:", instr)
				sys.exit(1)
		elif instr["mode"] in ["p"]:
			if instr["mnemonic"] in ["CMPR"]:
				mnemonic = instr["mnemonic"]
				instr["defs"] += ["NZ", "N", "Z", "V"]
			else:
				print("Compare postbyte instruction not handled:", instr)
		else:
			print("Compare instruction mode not handled:", instr)
			sys.exit(1)
	elif instr["function"] == "r":
		instr["lets"]["mayLoad"] = "false"
		instr["lets"]["mayStore"] = "false"
		if instr["mnemonic"] == "BRA" or instr["mnemonic"] == "LBRA":
			instr["ins"] = ["pcrel16:$tgt"]
			instr["lets"]["isBranch"] = "true"
			instr["lets"]["isTerminator"] = "true"
			instr["lets"]["isBarrier"] = "true"
		elif instr["mnemonic"] == "BRN":
			instr["ins"] = ["pcrel16:$tgt"]
			instr["lets"]["isBranch"] = "true"
			instr["lets"]["isTerminator"] = "true"
		elif instr["mnemonic"] == "B$COND":
			instr["mnemonic"] = "B"
			instr["name"] = instr["mnemonic"] + instr["mode"]
			instr["ins"] = ["condcode:$cond","pcrel8:$tgt"]
			instr["lets"]["isBranch"] = "true"
			instr["lets"]["isTerminator"] = "true"
		elif instr["mnemonic"] == "LB$COND":
			instr["mnemonic"] = "LB"
			instr["name"] = instr["mnemonic"] + instr["mode"]
			instr["ins"] = ["condcode:$cond","pcrel16:$tgt"]
			instr["lets"]["isBranch"] = "true"
			instr["lets"]["isTerminator"] = "true"
		elif instr["mnemonic"] == "BSR":
			instr["ins"] = ["pcrel8:$tgt"]
			instr["lets"]["isCall"] = "true"
			instr["uses"] = ["SS"]
		elif instr["mnemonic"] == "LBSR":
			instr["ins"] = ["pcrel16:$tgt"]
			instr["lets"]["isCall"] = "true"
			instr["uses"] = ["SS"]
		elif instr["mode"] == "r":
			instr["lets"]["isReturn"] = "true"
			instr["lets"]["isTerminator"] = "true"
			instr["lets"]["isBarrier"] = "true"
		else:
			print("Branch instruction not handled:", instr)
	elif instr["function"] == "j":
		if instr["mnemonic"] == "JMP":
			instr["lets"]["isBranch"] = "true"
			instr["lets"]["isBarrier"] = "true"
			instr["lets"]["isTerminator"] = "true"
		elif instr["mnemonic"] == "JSR":
			instr["lets"]["isCall"] = "true"
		elif instr["mnemonic"] == "RTS" or instr["mnemonic"] == "RTI":
			instr["lets"]["isReturn"] = "true"
			instr["lets"]["isBarrier"] = "true"
			instr["lets"]["isTerminator"] = "true"
		else:
			print("Jump instruction not handled:", instr)
	elif instr["function"] == "l":
		if instr["mnemonic"][:-1] == "LD":
			reg = instr["mnemonic"][-1:]
			instr["lets"]["mayLoad"] = "true"
			instr["defs"] = ["NZ", "N", "Z", "V"] + defs(reg)
		if instr["mnemonic"][:-1] == "CLR":
			reg = instr["mnemonic"][-1:]
			if reg in "ABDEFWQ":
				instr["defs"] = ["NZ", "N", "Z", "V"] + defs(reg)
			else:
				print("Clear register not handled:", instr)
	elif instr["function"] == "s":
		if instr["mnemonic"][:-1] == "ST":
			reg = instr["mnemonic"][-1:]
			instr["defs"] = ["NZ", "N", "Z", "V"]
			instr["lets"]["mayStore"] = "true"
		elif instr["mnemonic"] == "CLR":
			instr["defs"] = ["NZ", "N", "Z", "V"]
			instr["lets"]["mayStore"] = "true"
	elif instr["function"] == "ss":
		if instr["mnemonic"][0:4] in ["PSHS", "PULS"]:
			instr["uses"] += ["SS"]
			instr["defs"] += ["SS"]
		elif instr["mnemonic"][0:4] in ["PSHU", "PULU"]:
			instr["uses"] += ["SU"]
			instr["defs"] += ["SU"]
		else:
			print("Stack operation not handled", instr)
	elif instr["function"] == "m":
		pass
	elif instr["function"] == "x":
		pass
	elif instr["function"] == "b":
		pass
	else:
		print("Unknown instruction function '{}'".format(instr["function"]))

	if instr["mnemonic"][:-1] == "LEA":
		reg = instr["mnemonic"][-1:]
		if reg in "XY":
			instr["defs"] = ["I" + reg]
		elif reg in "SU":
			instr["defs"] = ["S" + reg]
		else:
			print("Strange LEAr instruction:", instr)

	instr["indexmode"] = ''

	if instr["mode"] == "a":
		instr["addressmode"] = "Accumulator"
		instr["params"] = [ ((7, 0), "Opc") ]
		insert_instruction(instr)
	elif instr["mode"] == "r":
		instr["addressmode"] = "Return"
		instr["params"] = [ ((7, 0), "Opc") ]
		insert_instruction(instr)
	elif instr["mode"] == "x":
		instr["addressmode"] = "Inherent"
		instr["params"] = [ ((7, 0), "Opc") ]
		insert_instruction(instr)
	elif instr["mode"] == "bd":
		assert instr["hd6309"] == 1
		assert instr["size"] == 4, instr
		assert instr["page"] == 3, instr
		instr["addressmode"] = "BitDirect"
		instr["ins"] += ["BIT8:$reg","imm3:$dstbit","imm3:$srcbit","addr8:$addr"]
		instr["outs"] += ["BIT8:$dst"]
		instr["lets"]["Constraints"] = ["$dst = $reg"]
		instr["params"] = [ ((23, 16), "addr{7-0}"), ((15, 14), "reg{1-0}"), ((13, 11), "srcbit"), ((10, 8), "dstbit"), ((7, 0), "Opc") ]
		insert_instruction(instr)
	elif instr["mode"] == "id":
		assert instr["hd6309"] == 1
		assert instr["size"] == 3, instr
		assert instr["page"] == 1, instr
		instr["addressmode"] = "ImmediateDirect"
		instr["ins"] += ["addr8:$addr","i8imm:$val"]
		instr["params"] = [ ((23, 16), "addr{7-0}"), ((15, 8), "val{7-0}"), ((7, 0), "Opc") ]
		insert_instruction(instr)
	elif instr["mode"] == "ie":
		assert instr["hd6309"] == 1
		assert instr["size"] == 4, instr
		assert instr["page"] == 1, instr
		instr["addressmode"] = "ImmediateExtended"
		instr["ins"] += ["addr16:$addr","i8imm:$val"]
		instr["params"] = [ ((31, 24), "addr{7-0}"), ((23, 16), "addr{15-8}"), ((15, 8), "val{7-0}"), ((7, 0), "Opc") ]
		insert_instruction(instr)
	elif instr["mode"] == "s":
		instr["addressmode"] = "Stack"
		instr["ins"] += ["reglist:$regs", "variable_ops"]
		instr["params"] = [ ((15, 8), "regs"), ((7, 0), "Opc") ]
		insert_instruction(instr)
	elif instr["mode"] == "p":
		instr["addressmode"] = "RegisterPair"
		if instr["mnemonic"] == "TFR":
			instr["addressmode"] += "Copy"
			instr["ins"] += ["anyregister:$reg1"]
			instr["outs"] += ["anyregister:$reg2"]
		elif instr["mnemonic"] == "EXG":
			instr["addressmode"] += "Swap"
			instr["ins"] += ["anyregister:$reg1","anyregister:$reg2"]
			instr["outs"] += ["anyregister:$dst2","anyregister:$dst1"]
			instr["lets"]["Constraints"] += ["$dst2 = $reg1","$dst1 = $reg2"]
		elif instr["mnemonic"] == "CMPR":
			instr["addressmode"] += "Compare"
			instr["ins"] += ["anyregister:$reg1","anyregister:$reg2"]
			instr["outs"] += []
			instr["lets"]["isCompare"] = "true"
		elif instr["mnemonic"][-1] == "R":
			instr["addressmode"] += "Arithmetic"
		else:
			print("Unknown register pair instruction ", instr)
			sys.exit(0)
		instr["params"] = [ ((15, 12), "reg1"), ((11, 8), "reg2"), ((7, 0), "Opc") ]
		insert_instruction(instr)
	elif instr["mode"] == "pp":
		instr["addressmode"] = "BlockRegisterPair"
		instr["ins"] += ["anyregister:$reg1","anyregister:$reg2"]
		instr["outs"] += ["anyregister:$dst"]
		instr["name"] = instr["mnemonic"] + str(instr["opcode"] - 0x38) + instr["mode"]
		instr["addressmode"] += str(instr["opcode"] - 0x38)
		instr["params"] = [ ((15, 12), "reg1"), ((11, 8), "reg2"), ((7, 0), "Opc") ]
		insert_instruction(instr)
	elif instr["mode"] == "d":
		instr["addressmode"] = "Direct"
		instr["ins"] += ["addr8:$addr"]
		instr["params"] = [ ((15, 8), "addr{7-0}"), ((7, 0), "Opc") ]
		insert_instruction(instr)
	elif instr["mode"] == "e":
		instr["addressmode"] = "Extended"
		instr["ins"] += ["addr16:$addr"]
		instr["params"] = [ ((23, 16), "addr{7-0}"), ((15, 8), "addr{15-8}"), ((7, 0), "Opc") ]
		insert_instruction(instr)
	elif instr["mode"] == "i8":
		instr["addressmode"] = "Immediate8Bit"
		instr["ins"] += ["i8imm:$val"]
		instr["params"] = [ ((15, 8), "val{7-0}"), ((7, 0), "Opc") ]
		insert_instruction(instr)
	elif instr["mode"] == "i16":
		instr["addressmode"] = "Immediate16Bit"
		instr["ins"] += ["i16imm:$val"]
		instr["params"] = [ ((23, 16), "val{7-0}"), ((15, 8), "val{15-8}"), ((7, 0), "Opc") ]
		insert_instruction(instr)
	elif instr["mode"] == "i32":
		instr["addressmode"] = "Immediate32Bit"
		instr["ins"] += ["i32imm:$val"]
		instr["params"] = [ ((39, 32), "val{7-0}"), ((31, 24), "val{15-8}"), ((23, 16), "val{23-16}"), ((15, 8), "val{31-24}"), ((7, 0), "Opc") ]
		insert_instruction(instr)
	elif instr["mode"] == "b" or instr["mode"] == "lb":
		assert instr["page"] == 1, instr
		if instr["mode"] == "b":
			instr["addressmode"] = "ShortBranch"
		else:
			instr["addressmode"] = "LongBranch"
		if instr["mnemonic"] == "BRA" or instr["mnemonic"] == "BRN" or instr["mnemonic"] == "BSR":
			assert instr["size"] == 2, instr
			instr["params"] = [ ((15, 8), "tgt{7-0}"), ((7, 0), "Opc") ]
		else:
			assert instr["size"] == 3, instr
			instr["params"] = [ ((23, 16), "tgt{7-0}"), ((15, 8), "tgt{15-8}"), ((7, 0), "Opc") ]
		insert_instruction(instr)
	elif instr["mode"] == "bc" or instr["mode"] == "lbc":
		if instr["mode"] == "bc":
			instr["addressmode"] = "ShortConditionalBranch"
			instr["params"] = [ ((15, 8), "tgt{7-0}"), ((7, 4), "Opc{3-0}"), ((3, 0), "cond") ]
		else:
			instr["addressmode"] = "LongConditionalBranch"
			instr["params"] = [ ((23, 16), "tgt{7-0}"), ((15, 8), "tgt{15-8}"), ((7, 4), "Opc{3-0}"), ((3, 0), "cond") ]
		if instr["page"] == 1:
			assert instr["size"] == 2, instr
		else:
			assert instr["size"] == 4, instr
		insert_instruction(instr)
	elif instr["mode"] == "ii":
		assert instr["hd6309"] == 1
		instr["addressmode"] = "ImmediateIndexed"
		instr["ins"] += ["i8imm:$val"]
		insert_instruction(instr)
	elif instr["mode"] == "i":
		instr["addressmode"] = "Indexed"
		insert_instruction(instr)
	else:
		print("Unknown instruction mode '{}'".format(instr["mode"]), instr)
		sys.exit(0)

if False:
	print("OINQUE! Early termination! No files written!")
	sys.exit(0)

for page in range(1):
	# Immediate Indexed
	i = 0
	multiclass = "MC6809ImmediateIndexed_P"+ str(page + 1)
	MultiClass[multiclass] = { }
	for indexmode in IndexModes:
		instform = {}
		instform["multiclass"] = multiclass
		instform["indexnumber"] = i
		i += 1
		instform["page"] = page + 1
		instform["mode"] = "ii"
		instform["function"] = ""
		instform["addressmode"] = "ImmediateIndexed" + indexmode[0]
		instform["indexmode"] = "_" + indexmode[1]
		instform["operand"] = ' '.join(["$val", ";"] + indexmode[2])
		instform["operandsize"] = indexmode[4] + 1
		im = indexmode[3]
		newmode = []
		for m in im:
			if m[0][1] == -1:
				m = ((m[0][0] + 8, -1), m[1])
			else:
				m = ((m[0][0] + 8, m[0][1] + 8), m[1])
			newmode += [ m ]
		instform["params"] = newmode + [ ((15, 8), "val{7-0}"), ((7, 0), "Opc") ]
		instform["ins"] = [ "i8imm:$val" ] + indexmode[5].split(',')
		instform["outs"] = []
		instform["hd6309"] = True
		instform["defs"] = indexmode[7]
		instform["uses"] = indexmode[8]
		instform["lets"] = {}
		instform["lets"]["Constraints"] = []
		instform["lets"]["Predicates"] = []
		if instform["hd6309"]:
			instform["lets"]["Predicates"] += ["IsHD6309"]
		insert_indexmode(instform)

for page in range(3):
	# Indexed
	i = 0
	multiclass = "MC6809Indexed_P"+ str(page + 1)
	MultiClass[multiclass] = { }
	for indexmode in IndexModes:
		instform = {}
		instform["multiclass"] = multiclass
		instform["indexnumber"] = i
		i += 1
		instform["page"] = page + 1
		instform["mode"] = "i"
		instform["function"] = ""
		instform["addressmode"] = "Indexed" + indexmode[0]
		instform["indexmode"] = "_" + indexmode[1]
		instform["operand"] = ' '.join(indexmode[2])
		instform["operandsize"] = indexmode[4]
		instform["params"] = indexmode[3] + [ ((7, 0), "Opc") ]
		instform["hd6309"] = indexmode[6]
		instform["ins"] = indexmode[5].split(',')
		instform["outs"] = []
		instform["defs"] = indexmode[7]
		instform["uses"] = indexmode[8]
		instform["lets"] = {}
		instform["lets"]["Constraints"] = []
		instform["lets"]["Predicates"] = []
		if instform["hd6309"]:
			instform["lets"]["Predicates"] += ["IsHD6309"]
		insert_indexmode(instform)

with Generate("MC6809InstrInfo.td", marker_start="// MRVM START MARKER 1", marker_finish="// MRVM END MARKER 1") as ii:
	for mck in sorted(MultiClass):
		ii.write("// ============================== PAGE {} ==================================\n".format(mck))
		ii.write("multiclass {}<string Mnemonic, Opcode opcode, dag outdag, list<Register> defsin, list<Register> usesin> {{\n".format(mck))
		for mc in sorted(MultiClass[mck]):
			ii.write(MultiClass[mck][mc] + "\n")
		ii.write("}\n")
	ii.write("// ============================== END PAGES ==================================\n")

with Generate("MC6809InstrInfo.td", marker_start="// MRVM START MARKER 2", marker_finish="// MRVM END MARKER 2") as ii:
	for page in Instructions:
		for instr in page:
			if instr["mode"] == "i" or instr["mode"] == "ii":
				if instr["lets"]:
					ii.write("let " + ", ".join(k + " = " + instr["lets"][k] for k in sorted(instr["lets"])) + " in {\n")
					ii.write('  defm {name} : MC6809{addressmode}_P{page}<"{lcmnemonic}", Opcode<0x{opcode:02X}, {page}>, {outsformatted}, [{defslist}], [{useslist}]>;\n'.format(**instr))
					ii.write("}\n\n")
				else:
					ii.write('defm {name} : MC6809{addressmode}_P{page}<"{lcmnemonic}", Opcode<0x{opcode:02X}, {page}>, {outsformatted}, [{defslist}], [{useslist}]>;\n\n'.format(**instr))
			else:
				if instr["lets"]:
					ii.write('def {name} : MC6809{addressmode}_P{page}<{outsformatted}, "{lcmnemonic}", Opcode<0x{opcode:02X},{page}>, [{defslist}], [{useslist}]> {{\n'.format(**instr))
					for l in instr["lets"]:
						ii.write("  let {} = {};\n".format(l, instr["lets"][l]))
					ii.write('}\n\n')
				else:
					ii.write('def {name} : MC6809{addressmode}_P{page}<{outsformatted}, "{lcmnemonic}", Opcode<0x{opcode:02X},{page}>, [{defslist}], [{useslist}]>;\n\n'.format(**instr))

pagecode = {1:"    ", 2:"0x10", 3:"0x11"}
conds = {1:"rn", 2:"hi", 3:"ls", 4:"hs", 5:"lo", 6:"ne", 7:"eq", 8:"vc", 9:"vs", 10:"pl", 11:"mi", 12:"ge", 13:"lt", 14:"gt", 15:"le"}

with Generate("MC6809InstrDisassembler6809Tests.txt", marker_start="# MRVM START MARKER", marker_finish="# MRVM END MARKER") as tt6809, Generate("MC6809InstrDisassembler6309Tests.txt", marker_start="# MRVM START MARKER", marker_finish="# MRVM END MARKER") as tt6309:
	for page in Instructions:
		for instr in page:
			hd6309 = instr["hd6309"] == 1
			if hd6309:
				tt = tt6309
			else:
				tt = tt6809
			instr["pagecode"] = pagecode[instr["page"]]
			if instr["mode"] == "i" or instr["mode"] == "ii":
				if instr["mode"] not in ["i", "ii"]:
					print("OINQUE!! Diassembler tests indexed address mode '{mode}' not handled!".format(**instr))
				if instr["mode"] == "i":
					for indexmode in IndexModes:
						indexhd6309 = bool(indexmode[6])
						if hd6309 or indexhd6309:
							tt = tt6309
						else:
							tt = tt6809
						instr["operand"] = ' '.join(indexmode[2])
						postbyte = indexmode[9]
						if postbyte.needs_index():
							postbyte.set_index_random()
						if postbyte.needs_offset():
							postbyte.set_offset_random()
						instr["symbolic_operand"] = postbyte.set_symbolic_operand(instr["operand"])
						instr["postbyte_string"] = ' '.join([ f"0x{b:02X}" for b in postbyte.get_bytes() ])
						tt.write("{pagecode} 0x{opcode:02X} {postbyte_string:15s}      #\t{lcmnemonic}\t{symbolic_operand}\n".format(**instr))
				if instr["mode"] == "ii":
					for indexmode in IndexModes:
						indexhd6309 = bool(indexmode[6])
						assert hd6309
						instr["operand"] = "$val ; " + ' '.join(indexmode[2])
						postbyte = indexmode[9]
						if postbyte.needs_index():
							postbyte.set_index_random()
						if postbyte.needs_offset():
							postbyte.set_offset_random()
						instr["symbolic_operand"] = postbyte.set_symbolic_operand(instr["operand"])
						instr["postbyte_string"] = ' '.join([ f"0x{b:02X}" for b in postbyte.get_bytes() ])
						tt.write("{pagecode} 0x{opcode:02X} 0x?? {postbyte_string:15s} #\t{lcmnemonic}\t{symbolic_operand}\n".format(**instr))
			else:
				if instr["mode"] not in ["d", "id", "ie", "x", "lb", "lbc", "i8", "i16", "i32", "p", "pp", "b", "bc", "s", "r", "a", "e", "bd"]:
					print("OINQUE!! Diassembler tests non-indexed address mode '{mode}' not handled!".format(**instr))
				if instr["mode"] == "d":
					tt.write("{pagecode} 0x{opcode:02X} 0x00                 #\t{lcmnemonic}\t<$00       {operand}\n".format(**instr))
				if instr["mode"] == "e":
					tt.write("{pagecode} 0x{opcode:02X} 0x00 0x00            #\t{lcmnemonic}\t>$0000     {operand}\n".format(**instr))
				if instr["mode"] == "id":
					tt.write("{pagecode} 0x{opcode:02X} 0x00 0x00            #\t{lcmnemonic}\t$00;<$00   {operand}\n".format(**instr))
				if instr["mode"] == "ie":
					tt.write("{pagecode} 0x{opcode:02X} 0x00 0x00 0x00       #\t{lcmnemonic}\t$00;>$0000 {operand}\n".format(**instr))
				if instr["mode"] in ["x", "r", "a"]:
					tt.write("{pagecode} 0x{opcode:02X}                      #\t{lcmnemonic}             {operand}\n".format(**instr))
				if instr["mode"] == "lb":
					tt.write("{pagecode} 0x{opcode:02X} 0x00 0x00            #\t{lcmnemonic}\t$0000      {operand}\n".format(**instr))
				if instr["mode"] == "i8":
					tt.write("{pagecode} 0x{opcode:02X} 0x00                 #\t{lcmnemonic}\t#$00       {operand}\n".format(**instr))
				if instr["mode"] == "i16":
					tt.write("{pagecode} 0x{opcode:02X} 0x00 0x00            #\t{lcmnemonic}\t#$0000     {operand}\n".format(**instr))
				if instr["mode"] == "i32":
					tt.write("{pagecode} 0x{opcode:02X} 0x00 0x00 0x00 0x00  #\t{lcmnemonic}\t#$0000     {operand}\n".format(**instr))
				if instr["mode"] == "p":
					tt.write("{pagecode} 0x{opcode:02X} 0x89                 #\t{lcmnemonic}\ta,b        {operand}\n".format(**instr))
				if instr["mode"] == "pp":
					tt.write("{pagecode} 0x{opcode:02X} 0x12                 #\t{lcmnemonic}\tx,y        {operand}\n".format(**instr))
				if instr["mode"] == "bd":
					tt.write("{pagecode} 0x{opcode:02X} 0x69 0x00            #\t{lcmnemonic}\ta,5,1,<$00 {operand}\n".format(**instr))
				if instr["mode"] == "b":
					tt.write("{pagecode} 0x{opcode:02X} 0x00                 #\t{lcmnemonic}\t#00        {operand}\n".format(**instr))
				if instr["mode"] == "bc":
					for cond in range(2, 16):
						instr["conditioncode"] = cond
						instr["lcmnemonic"] = "b" + conds[cond]
						tt.write("{pagecode} 0x{opcode:X}{conditioncode:X} 0x00                 #\t{lcmnemonic}\t$00        {operand}\n".format(**instr))
					instr["lcmnemonic"] = "b$cond"
				if instr["mode"] == "lbc":
					for cond in range(1, 16):
						instr["conditioncode"] = cond
						instr["lcmnemonic"] = "lb" + conds[cond]
						tt.write("{pagecode} 0x{opcode:X}{conditioncode:X} 0x00 0x00            #\t{lcmnemonic}\t$0000      {operand}\n".format(**instr))
					instr["lcmnemonic"] = "lb$cond"
				if instr["mode"] == "s":
					tt.write("{pagecode} 0x{opcode:02X} 0xFF                #\t{lcmnemonic}\tcc,a,b,dp,x,y,u,pc {operand}\n".format(**instr))

with Generate("MC6809InstrFormats.td", marker_start="// MRVM START MARKER 1", marker_finish="// MRVM END MARKER 1") as fi:
	for k in sorted(AddressMode):
		v = AddressMode[k]
		fi.write("class {addressmode}<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {{\n".format(**v))
		if  "pcrel8:$tgt" in v["insformatted"] or "pcrel16:$tgt" in v["insformatted"]:
			fi.write('  bits<16> tgt;\n')
		if "condcode:$cond" in v["insformatted"]:
			fi.write('  bits<4> cond;\n')
		if "INDEX16:$ireg" in v["insformatted"]:
			fi.write('  bits<2> ireg;\n')
		if "pcrel8:$offset" in v["insformatted"] or "pcrel16:$offset" in v["insformatted"] or "offset5:$offset" in v["insformatted"] or "offset8:$offset" in v["insformatted"] or "offset16:$offset" in v["insformatted"]:
			fi.write('  bits<16> offset;\n')
		if "addr8:$addr" in v["insformatted"] or "addr16:$addr" in v["insformatted"]:
			fi.write('  bits<16> addr;\n')
		if "i8imm:$val" in v["insformatted"] or v["mode"] == "ii":
			fi.write('  bits<8> val;\n')
		if "i16imm:$val" in v["insformatted"]:
			fi.write('  bits<16> val;\n')
		if "i32imm:$val" in v["insformatted"]:
			fi.write('  bits<32> val;\n')
		if "anyregister:$reg1" in v["insformatted"]:
			fi.write('  bits<4> reg1;\n')
			fi.write('  bits<4> reg2;\n')
		if "reglist:$regs" in v["insformatted"]:
			fi.write('  bits<8> regs;\n')
		if "BIT8:$reg" in v["insformatted"]:
			fi.write('  bits<2> reg;\n')
			fi.write('  bits<3> srcbit;\n')
			fi.write('  bits<3> dstbit;\n')
		for inst in v["inst"]:
			fi.write('  {}\n'.format(inst))
		fi.write('  let DecoderNamespace = "_Page_" # Page # "_Size_";\n'.format(**v))
		fi.write('  let Mode = "{mode}{indexmode}";\n'.format(**v))
		fi.write('  let OperandSize = {operandsize};\n'.format(**v))
		fi.write('  let Size = !add(OperandSize, OpcodeSize);\n'.format(**v))
		fi.write('  let InOperandList = {insformatted};\n'.format(**v))
		fi.write('  let AsmString = !strconcat(mnemonic, "{operand}");\n'.format(**v))
		fi.write('  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);\n')
		fi.write("}\n\n")

with Generate("MC6809InstrFormats.td", marker_start="// MRVM START MARKER 2", marker_finish="// MRVM END MARKER 2") as fi:
	for k in sorted(InstrFormat):
		v = InstrFormat[k]
		fi.write('class {}<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>\n'.format(k))
		fi.write('  : {addressmode}<mnemonic, opc> {{\n'.format(**v))
		fi.write('    let Defs = defs;\n')
		fi.write('    let Uses = uses;\n')
		fi.write('    let OutOperandList = outs;\n')
		fi.write('}\n\n')

DecoderTables = sorted(list(DecoderTables))
ModeEnums = sorted(list(ModeEnums))
print("There are {} distinct addressing modes".format(len(ModeEnums)))

with Generate("Disassembler/MC6809Disassembler.cpp") as dt:
	dt.write("  unsigned DecoderTableSize = {};\n".format(len(DecoderTables)))
	dt.write("  struct DecoderTableList {\n")
	dt.write("    const uint8_t *Table;\n")
	dt.write("    unsigned Size;\n")
	dt.write("  }} DecoderTable[{}] = {{\n".format(len(DecoderTables)))
	for table in DecoderTables:
		dt.write('    {{ {}, {} }},\n'.format(*table))
	dt.write("  };\n")

#!/usr/bin/env python3

import sys
import copy

from instructions import RawInstructions
from indexmodes import IndexModes

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
	instr["lcmnemonic"] = instr["mnemonic"].lower()
	instr["name"] = instr["mnemonic"] + instr["mode"]
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

	assert instr["mnemonic"] != "", instr
	assert instr["page"] >= 1 and instr["page"] <= 3, instr
	if (len(instr["mnemonic"]) == 3 and instr["mnemonic"][:2] == "LD") or (len(instr["mnemonic"]) == 4 and instr["mnemonic"][:3] == "CLR"):
		assert instr["function"] == "l", instr
		dstreg = instr["mnemonic"][-1]
		assert dstreg in "ABDEFWQXYUS", instr
	if instr["mnemonic"][:3] == "LEA":
		assert instr["function"] == "l", instr
		destreg = instr["mnemonic"][-1]
		assert destreg in "XYUS", instr
	if len(instr["mnemonic"]) == 3 and instr["mnemonic"][:2] == "ST":
		assert instr["function"] == "s", instr
		srcreg = instr["mnemonic"][-1]
		assert srcreg in "ABDEFWQXYUS", instr
	if instr["function"] == "l":
		assert (len(instr["mnemonic"]) == 3 and instr["mnemonic"][:2] == "LD") or (len(instr["mnemonic"]) == 4 and instr["mnemonic"][:3] == "CLR") or (len(instr["mnemonic"]) == 4 and instr["mnemonic"][:3] == "LEA") or instr["mnemonic"] == "LDBT" or instr["mnemonic"] == "LDMD", instr
	if instr["function"] == "s":
		assert (len(instr["mnemonic"]) == 3 and instr["mnemonic"][:2] == "ST") or instr["mnemonic"] == "CLR" or instr["mnemonic"] == "STBT", instr
	if instr["function"] == "a":
			pass
	if instr["mnemonic"] == "CMP" or instr["mnemonic"][:3] == "CMP" or instr["mnemonic"] == "TST" or instr["mnemonic"][:3] == "TST":
		assert instr["function"] == "c", instr
	if instr["function"] == "c":
		assert instr["mnemonic"] == "CMP" or instr["mnemonic"][:3] == "CMP" or instr["mnemonic"] == "TST" or instr["mnemonic"][:3] == "TST", instr
	if instr["mode"] == "a":
		assert instr["operand"] == "", instr
	if instr["mode"] == "s":
		assert instr["function"] == "ss", instr
		assert instr["operand"] == "$regs", instr
	if instr["mode"] == "d":
		assert instr["operand"] == "< $addr", instr
	if instr["mode"] == "bd":
		assert instr["operand"] == "$dst , $srcbit , $dstbit , < $addr", instr
	if instr["mode"] == "id":
		assert instr["operand"] == "# $val ; < $addr", instr
	if instr["mode"] == "ie":
		assert instr["operand"] == "# $val ; $addr", instr
	if instr["mode"] == "i8":
		assert instr["operand"] == "# $val", instr
	if instr["mode"] == "i16":
		assert instr["operand"] == "# $val", instr
	if instr["mode"] == "i32":
		assert instr["operand"] == "# $val", instr
	if instr["mode"] == "d":
		assert instr["operand"] == "< $addr", instr
	if instr["mode"] == "e":
		assert instr["operand"] == "$addr", instr
	if instr["mode"] == "p":
		assert instr["mnemonic"] == "EXG" or instr["mnemonic"] == "TFR" or instr["mnemonic"][-1] == 'R', instr
		assert instr["operand"] == "$reg1 , $reg2", instr
	if instr["mode"] == "pp":
		assert instr["mnemonic"] == "TFM", instr
		assert instr["operand"] == "$reg1 + , $reg2 +" or instr["operand"] == "$reg1 - , $reg2 -" or instr["operand"] == "$reg1 + , $reg2" or instr["operand"] == "$reg1 , $reg2 +", instr
	for i in instr["ins"]:
		assert len(i) == 0, instr
	for o in instr["outs"]:
		assert len(o) == 0, instr

	if instr["function"] == "r":
		instr["lets"]["mayLoad"] = "false";
		instr["lets"]["mayStore"] = "false";
		if instr["mnemonic"] == "LBRA":
			instr["ins"] = ["label:$tgt"]
			instr["lets"]["isBranch"] = "true";
			instr["lets"]["isTerminator"] = "true";
			instr["lets"]["isBarrier"] = "true";
		elif instr["mnemonic"] == "B$COND":
			instr["mnemonic"] = "B"
			instr["name"] = instr["mnemonic"] + instr["mode"]
			instr["ins"] = ["condcode:$cond","label:$tgt"]
			instr["lets"]["isBranch"] = "true";
			instr["lets"]["isTerminator"] = "true";
		elif instr["mnemonic"] == "LB$COND":
			instr["mnemonic"] = "LB"
			instr["name"] = instr["mnemonic"] + instr["mode"]
			instr["ins"] = ["condcode:$cond","label:$tgt"]
			instr["lets"]["isBranch"] = "true";
			instr["lets"]["isTerminator"] = "true";
		elif instr["mnemonic"] == "BSR":
			instr["ins"] = ["label:$tgt"]
			instr["lets"]["isCall"] = "true";
			instr["defs"] = ["SU"]
			instr["uses"] = ["SS"]
		elif instr["mnemonic"] == "LBSR":
			instr["ins"] = ["label:$tgt"]
			instr["lets"]["isCall"] = "true";
			instr["defs"] = ["SU"]
			instr["uses"] = ["SS"]
		elif instr["mode"] == "r":
			instr["lets"]["isReturn"] = "true";
			instr["lets"]["isTerminator"] = "true";
			instr["lets"]["isBarrier"] = "true";
		else:
			print("Branch instruction not handled:", instr)
	elif instr["function"] == "j":
		if instr["mnemonic"] == "JMP":
			instr["lets"]["isBranch"] = "true";
		elif instr["mnemonic"] == "JSR":
			instr["lets"]["isCall"] = "true";
		elif instr["mnemonic"] == "RTS" or instr["mnemonic"] == "RTI":
			instr["lets"]["isReturn"] = "true";
			instr["lets"]["isBarrier"] = "true";
			instr["lets"]["isTerminator"] = "true";
		else:
			print("Jump instruction not handled:", instr)
	elif instr["function"] == "c":
		instr["lets"]["isCompare"] = "true";
		instr["defs"] = ["NZVC"]
		if instr["mode"] == "a":
			if instr["mnemonic"][:-1] == "TST":
				reg = instr["mnemonic"][-1:]
				if reg in "ABDEFW":
					instr["uses"] += ["A" + reg]
				else:
					print("Test register not handled:", instr)
			else:
				print("Test mode not handled", instr)
		else:
			if instr["mnemonic"][:-1] in ["TST", "CMP"]:
				reg = instr["mnemonic"][-1:]
				if reg in "ABDEFW":
					instr["uses"] += ["A" + reg]
				elif reg in "XY":
					instr["uses"] += ["I" + reg]
				elif reg in "SU":
					instr["uses"] += ["S" + reg]
				else:
					print("Compare register not handled:", instr)
			elif instr["mnemonic"] == "CMPR":
				# instr["ins"] += ["anyregister:$reg1","anyregister:$reg2"]
				pass
			else:
				print("Compare mnemonic not handled:", instr)
	elif instr["function"] == "l":
		if instr["mnemonic"][:-1] == "LD":
			reg = instr["mnemonic"][-1:]
			instr["lets"]["mayLoad"] = "true";
			instr["defs"] = ["NZVC"]
			if reg in "ABDEFWQ":
				instr["outs"] = ["A" + reg + "c:$reg"]
			elif reg in "XY":
				instr["outs"] = ["I" + reg + "c:$reg"]
			elif reg in "SU":
				instr["outs"] = ["S" + reg + "c:$reg"]
			else:
				print("Load register not handled:", instr)
		if instr["mnemonic"][:-1] == "CLR":
			reg = instr["mnemonic"][-1:]
			instr["defs"] = ["NZVC"]
			instr["lets"]["mayLoad"] = "true";
			if reg in "ABDEFWQ":
				instr["outs"] = ["A" + reg + "c:$reg"]
			elif reg in "XY":
				instr["outs"] = ["I" + reg + "c:$reg"]
			elif reg in "SU":
				instr["outs"] = ["S" + reg + "c:$reg"]
			else:
				print("Load register not handled:", instr)
	elif instr["function"] == "s":
		if instr["mnemonic"][:-1] == "ST":
			reg = instr["mnemonic"][-1:]
			instr["defs"] = ["NZVC"]
			instr["lets"]["mayStore"] = "true";
			if reg in "ABDEFWQ":
				instr["ins"] = ["A" + reg + "c:$reg"]
			elif reg in "XY":
				instr["ins"] = ["I" + reg + "c:$reg"]
			elif reg in "SU":
				instr["ins"] = ["S" + reg + "c:$reg"]
			else:
				print("Store register not handled:", instr)
		elif instr["mnemonic"] == "CLR":
			instr["defs"] = ["NZVC"]
			instr["lets"]["mayStore"] = "true";
	elif instr["function"] == "a":
		instr["uses"] += ["NZVC"]
		instr["defs"] += ["NZVC"]
		if instr["mode"] == "a":
			if instr["mnemonic"][:-1] in ["ASL", "ASR", "COM", "DEC", "INC", "LSR", "NEG", "ROL", "ROR"]:
				reg = instr["mnemonic"][-1:]
				if reg in "ABDEFWQ":
					instr["uses"] += ["A" + reg]
					instr["defs"] += ["A" + reg]
				else:
					print("Arithmetic/Logical accumulator register not handled:", instr)
			else:
				print("Arithmetic accumulator instruction not handled:", instr)
		else:
			if instr["mnemonic"] in ["AIM", "OIM", "EIM", "TIM"]:
				instr["lets"]["mayLoad"] = "true";
				instr["lets"]["mayStore"] = "true";
			elif instr["mnemonic"] in ["ASL", "ASR", "COM", "DEC", "INC", "LSR", "NEG", "ROL", "ROR"]:
				instr["lets"]["mayLoad"] = "true";
				instr["lets"]["mayStore"] = "true";
			elif instr["mnemonic"][:-1] in ["ADD", "ADC", "SUB", "SBC", "AND", "OR", "EOR", "BIT"]:
				reg = instr["mnemonic"][-1:]
				instr["lets"]["mayLoad"] = "true";
				if reg != "R":
					if reg in "ABEFDW":
						instr["defs"] += ["A" + reg]
						instr["uses"] += ["A" + reg]
					else:
						print("Arithmetic operation not handled:", instr)
				else:
					instr["lets"]["Constraints"] += ["$dst = $reg2"]
			elif instr["mnemonic"] == "DIVD":
				instr["defs"] += ["AA","AB"]
				instr["uses"] += ["AD"]
			elif instr["mnemonic"] == "SEX":
				instr["defs"] += ["AD"]
				instr["uses"] += ["AB"]
			elif instr["mnemonic"] == "SEXW":
				instr["defs"] += ["AF"]
				instr["uses"] += ["AW"]
			elif instr["mnemonic"] == "ORCC" or instr["mnemonic"] == "ANDCC" or instr["mnemonic"] == "BITMD":
				# instr["ins"] += ["i8imm:$val"]
				pass
			else:
				print("Arithmetic instruction not handled:", instr)
	elif instr["function"] == "ss":
		pass
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
			instr["defs"] += ["I" + reg]
		elif reg in "US":
			instr["defs"] += ["S" + reg]
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
		instr["ins"] += ["reglist:$regs"]
		instr["params"] = [ ((15, 8), "regs"), ((7, 0), "Opc") ]
		insert_instruction(instr)
	elif instr["mode"] == "p":
		instr["addressmode"] = "RegisterPair"
		instr["ins"] += ["anyregister:$reg1","anyregister:$reg2"]
		instr["outs"] += ["anyregister:$dst"]
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
		if instr["mnemonic"] == "BSR":
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
		print("Unknown instruction mode '{}'".format(instr["mode"]))
		sys.exit(0)

for page in range(1):
	# Immediate Indexed
	i = 0
	multiclass = "MC6809ImmediateIndexed_P"+ str(page + 1)
	MultiClass[multiclass] = { }
	for mode in IndexModes:
		instform = {}
		instform["multiclass"] = multiclass
		instform["indexnumber"] = i
		i += 1
		instform["page"] = page + 1
		instform["mode"] = "ii"
		instform["function"] = ""
		instform["addressmode"] = "ImmediateIndexed" + mode[0]
		instform["indexmode"] = "_" + mode[1]
		instform["operand"] = ' '.join(["$val", ";"] + mode[2])
		instform["operandsize"] = mode[4] + 1
		im = mode[3]
		newmode = []
		for m in im:
			if m[0][1] == -1:
				m = ((m[0][0] + 8, -1), m[1])
			else:
				m = ((m[0][0] + 8, m[0][1] + 8), m[1])
			newmode += [ m ]
		instform["params"] = newmode + [ ((15, 8), "val{7-0}"), ((7, 0), "Opc") ]
		instform["ins"] = [ "i8imm:$val" ] + mode[5].split(',')
		instform["outs"] = []
		instform["hd6309"] = True
		instform["defs"] = mode[7]
		instform["uses"] = mode[8]
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
	for mode in IndexModes:
		instform = {}
		instform["multiclass"] = multiclass
		instform["indexnumber"] = i
		i += 1
		instform["page"] = page + 1
		instform["mode"] = "i"
		instform["function"] = ""
		instform["addressmode"] = "Indexed" + mode[0]
		instform["indexmode"] = "_" + mode[1]
		instform["operand"] = ' '.join(mode[2])
		instform["operandsize"] = mode[4]
		instform["params"] = mode[3] + [ ((7, 0), "Opc") ]
		instform["hd6309"] = mode[6]
		instform["ins"] = mode[5].split(',')
		instform["outs"] = []
		instform["defs"] = mode[7]
		instform["uses"] = mode[8]
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

with Generate("MC6809InstrFormats.td", marker_start="// MRVM START MARKER 1", marker_finish="// MRVM END MARKER 1") as fi:
	for k in sorted(AddressMode):
		v = AddressMode[k]
		fi.write("class {addressmode}<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {{\n".format(**v))
		if  "label:$tgt" in v["insformatted"]:
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
		fi.write('    let OutOperandList = outs;\n');
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

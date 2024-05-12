from ctypes import *
import random
import re

class ShortOffset(Structure):
	_fields_ = [
		("Offset", c_ubyte, 5),
		("Index", c_ubyte, 2),
		("TopBit", c_ubyte, 1),
	]

class WithIndex(Structure):
	_fields_ = [
		("Opcode", c_ubyte, 4),
		("Indirect", c_ubyte, 1),
		("Index", c_ubyte, 2),
		("TopBit", c_ubyte, 1),
	]

class Others(Structure):
	_fields_ = [
		("Opcode", c_ubyte, 7),
		("TopBit", c_ubyte, 1),
	]

class PostByte(Union):
	_fields_ = [
		("Byte", c_ubyte, 8),
		("ShortOffset", ShortOffset),
		("WithIndex", WithIndex),
		("Others", Others)
	]

class PostBytes:

	topbit = None
	index = None
	indirect = None
	opcode = None
	other_opcode = None
	offset = None
	pb = None
	operand = None

	def __init__(self, topbit, indirect=None, opcode=None, other_opcode=None):
		assert topbit in [0, 1]
		self.pb = PostByte()
		if topbit == 0:
			assert indirect is None
			assert opcode is None
			assert other_opcode is None
			self.topbit = 0
			self.pb.ShortOffset.TopBit = 0
		else:
			self.topbit = 1
			if other_opcode is None:
				assert indirect is not None and indirect in [0, 1]
				assert opcode is not None and opcode in range(16)

				self.pb.WithIndex.TopBit = 1

				self.indirect = indirect
				self.pb.WithIndex.Indirect = indirect

				self.opcode = opcode
				self.pb.WithIndex.Opcode = opcode

				if opcode in [ 0xC, 0xD ]:
					self.index = 0
					self.pb.WithIndex.Index = 0
			else:
				assert topbit == 1
				assert opcode is None
				assert other_opcode in range(128)
				self.indirect = 1
				self.other_opcode = other_opcode
				self.pb.Others.TopBit = 1
				self.pb.Others.Opcode = other_opcode

	def get_bytes(self):
		if self.topbit == 0:
			assert self.index is not None, "index register is not set for 5-bit offset postbyte"
			assert self.offset is not None, "offset is not set for 5-bit offset postbyte"
			return [ self.pb.Byte ]
		else:
			if self.other_opcode is None:
				assert self.index is not None, "index register is not set for postbyte"
				assert self.indirect is not None, "indirect bit is not set for postbyte"
				assert self.opcode is not None, "opcode is not set for postbyte"
				ret = [ self.pb.Byte ]
				if self.opcode in [ 0x8, 0xC ]:
					assert self.offset is not None
					ret += [ self.offset ]
				elif self.opcode in [ 0x9, 0xD ]:
					assert self.offset is not None
					ret += [ (self.offset >> 8), (self.offset & 0xFF) ]
				else:
					assert self.offset is None
				return ret
			else:
				ret = [ self.pb.Byte ]
				if self.other_opcode in [ 0x2F, 0x30, 0x1F ]:
					assert self.offset is not None
					ret += [ (self.offset >> 8), (self.offset & 0xFF) ]
				else:
					assert self.offset is None
				return ret

	def needs_index(self):
		return self.topbit == 0 or self.other_opcode is None and self.opcode not in [ 0xC, 0xD ]

	def set_index(self, index):
		assert self.needs_index()
		assert index in range(4)
		self.index = index
		if self.topbit == 0:
			self.pb.ShortOffset.Index = index
		else:
			self.pb.WithIndex.Index = index

	def set_index_random(self):
		assert self.needs_index()
		index = random.randrange(4)
		if self.topbit == 0:
			self.pb.ShortOffset.Index = index
		else:
			self.pb.WithIndex.Index = index
		self.index = index

	def needs_offset(self):
		return self.topbit == 0 or self.opcode in [ 0x8, 0x9, 0xC, 0xD ] or self.other_opcode in [ 0x2F, 0x30, 0x1F ]

	def set_offset(self, offset):
		assert self.needs_offset()
		if self.topbit == 0:
			assert offset in range(32)
			self.pb.ShortOffset.Offset = offset
		else:
			if self.opcode in [ 0x8, 0xC ]:
				assert offset in range(256)
			else:
				assert offset in range(65536)
		self.offset = offset

	def set_offset_random(self):
		assert self.needs_offset()
		if self.topbit == 0:
			while True:
				offset = random.randrange(32)
				if offset != 0:
					break
			self.offset = offset
			self.pb.ShortOffset.Offset = offset
		else:
			if self.opcode in [ 0x8, 0xC ]:
				while True:
					offset = random.randrange(256)
					if offset not in range(32):
						break
			else:
				while True:
					offset = random.randrange(65536)
					if offset not in range(256):
						break
			self.offset = offset

	def set_symbolic_operand(self, operand):
		self.operand = re.sub(' ', '', operand)
		if self.index is not None:
			self.operand = re.sub(r'\$ireg', ['x','y','u','s'][self.index], self.operand)
		if self.offset is not None:
			if self.topbit == 0:
				if self.offset > 15:
					offset = self.offset - 32
				else:
					offset = self.offset
				self.operand = re.sub(r'\$offset', f"{offset}", self.operand)
			else:
				if self.opcode in [ 0x8, 0xC ]:
					if self.offset > 127:
						offset = self.offset - 256
					else:
						offset = self.offset
					self.operand = re.sub(r'\$offset', f"{offset}", self.operand)
				else:
					if self.offset > 32767:
						offset = self.offset - 65536
					else:
						offset = self.offset
					self.operand = re.sub(r'\$offset', f"{offset}", self.operand)
				if self.other_opcode in [ 0x1F ]:
					self.operand = re.sub(r'\$addr', f"{offset}", self.operand)
		return self.operand

IndexModes = [
# Direct modes
[	"Offset0",
	"o0",
	[",", "$ireg"],
	[ ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b00100") ],
	1,
	"INDEX16:$ireg",
	False,
        [],
	[],
	PostBytes(1, 0, 0x4)
],
[	"Offset5",
	"o5",
	["$offset", ",", "$ireg"],
	[ ((15, -1), "0b0"), ((14, 13), "ireg"), ((12, 8), "offset{4-0}") ],
	1,
	"offset5:$offset,INDEX16:$ireg",
	False,
        [],
	[],
	PostBytes(0)
],
[	"Offset8",
	"o8",
	["$offset", ",", "$ireg"],
	[ ((23, 16), "offset{7-0}"), ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b01000") ],
	2,
	"offset8:$offset,INDEX16:$ireg",
	False,
        [],
	[],
	PostBytes(1, 0, 0x8)
],
[	"Offset16",
	"o16",
	["$offset", ",", "$ireg"],
	[ ((31, 24), "offset{7-0}"), ((23, 16), "offset{15-8}"), ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b01001") ],
	3,
	"offset16:$offset,INDEX16:$ireg",
	False,
        [],
	[],
	PostBytes(1, 0, 0x9)
],
[	"OffsetA",
	"oA",
	["a", ",", "$ireg"],
	[ ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b00110") ],
	1,
	"INDEX16:$ireg",
	False,
        [],
	["AA"],
	PostBytes(1, 0, 0x6)
],
[	"OffsetB",
	"oB",
	["b", ",", "$ireg"],
	[ ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b00101") ],
	1,
	"INDEX16:$ireg",
	False,
        [],
	["AB"],
	PostBytes(1, 0, 0x5)
],
[	"OffsetE",
	"oE",
	["e", ",", "$ireg"],
	[ ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b00111") ],
	1,
	"INDEX16:$ireg",
	True,
        [],
	["AE"],
	PostBytes(1, 0, 0x7)
],
[	"OffsetF",
	"oF",
	["f", ",", "$ireg"],
	[ ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b01010") ],
	1,
	"INDEX16:$ireg",
	True,
        [],
	["AF"],
	PostBytes(1, 0, 0xA)
],
[	"OffsetD",
	"oD",
	["d", ",", "$ireg"],
	[ ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b01011") ],
	1,
	"INDEX16:$ireg",
	False,
        [],
	["AD"],
	PostBytes(1, 0, 0xB)
],
[	"OffsetW",
	"oW",
	["w", ",", "$ireg"],
	[ ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b01110") ],
	1,
	"INDEX16:$ireg",
	True,
        [],
	["AW"],
	PostBytes(1, 0, 0xE)
],
[	"Inc1",
	"Inc1",
	[",", "$ireg", "+"],
	[ ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b00000") ],
	1,
	"INDEX16:$ireg",
	False,
        [],
	[],
	PostBytes(1, 0, 0x0)
],
[	"Inc2",
	"Inc2",
	[",", "$ireg", "++"],
	[ ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b00001") ],
	1,
	"INDEX16:$ireg",
	False,
        [],
	[],
	PostBytes(1, 0, 0x1)
],
[	"Dec1",
	"Dec1",
	[",", "-", "$ireg"],
	[ ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b00010") ],
	1,
	"INDEX16:$ireg",
	False,
        [],
	[],
	PostBytes(1, 0, 0x2)
],
[	"Dec2",
	"Dec2",
	[",", "--", "$ireg"],
	[ ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b00011") ],
	1,
	"INDEX16:$ireg",
	False,
        [],
	[],
	PostBytes(1, 0, 0x3)
],
[	"Offset8PC",
	"o8PC",
	["$offset", ",", "pc"],
	[ ((23, 16), "offset{7-0}"), ((15, -1), "0b1"), ((14, 13), "0b00"), ((12, 8), "0b01100") ],
	2,
	"pcrel8:$offset",
	False,
        [],
	[],
	PostBytes(1, 0, 0xC)
],
[	"Offset16PC",
	"o16PC",
	["$offset", ",", "pc"],
	[ ((31, 24), "offset{7-0}"), ((23, 16), "offset{15-8}"), ((15, -1), "0b1"), ((14, 13), "0b00"), ((12, 8), "0b01101") ],
	3,
	"pcrel16:$offset",
	False,
        [],
	[],
	PostBytes(1, 0, 0xD)
],
[	"Offset0W",
	"o0W",
	[",", "w"],
	[ ((15, 8), "0b10001111") ],
	1,
	"ACC16:$ireg",
	True,
        [],
	["AW"],
	PostBytes(1, other_opcode=0x0F)
],
[	"Offset16W",
	"o16W",
	["$offset", ",", "w"],
	[ ((31, 24), "offset{7-0}"), ((23, 16), "offset{15-8}"), ((15, 8), "0b10101111") ],
	3,
	"offset16:$offset,ACC16:$ireg",
	True,
        [],
	[],
	PostBytes(1, other_opcode=0x2F)
],
[	"IncW2",
	"IncW2",
	[",", "w", "++"],
	[ ((15, 8), "0b11001111") ],
	1,
	"ACC16:$ireg",
	True,
        ["AW"],
	["AW"],
	PostBytes(1, other_opcode=0x4F)
],
[	"DecW2",
	"DecW2",
	[",", "--", "w"],
	[ ((15, 8), "0b11101111") ],
	1,
	"ACC16:$ireg",
	True,
        ["AW"],
	["AW"],
	PostBytes(1, other_opcode=0x6F)
],
# Indirect modes
[	"Offset0Ind",
	"o0I",
	["[", ",", "$ireg", "]"],
	[ ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b10100") ],
	1,
	"INDEX16:$ireg",
	False,
        [],
	[],
	PostBytes(1, 1, 0x4)
],
[	"Offset8Ind",
	"o8I",
	["[", "$offset", ",", "$ireg", "]"],
	[ ((23, 16), "offset{7-0}"), ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b11000") ],
	2,
	"offset8:$offset,INDEX16:$ireg",
	False,
        [],
	[],
	PostBytes(1, 1, 0x8)
],
[	"Offset16Ind",
	"o16I",
	["[", "$offset", ",", "$ireg", "]"],
	[ ((31, 24), "offset{7-0}"), ((23, 16), "offset{15-8}"), ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b11001") ],
	3,
	"offset16:$offset,INDEX16:$ireg",
	False,
        [],
	[],
	PostBytes(1, 1, 0x9)
],
[	"OffsetAInd",
	"oAI",
	["[", "a", ",", "$ireg", "]"],
	[ ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b10110") ],
	1,
	"INDEX16:$ireg",
	False,
        [],
	["AA"],
	PostBytes(1, 1, 0x6)
],
[	"OffsetBInd",
	"oBI",
	["[", "b", ",", "$ireg", "]"],
	[ ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b10101") ],
	1,
	"INDEX16:$ireg",
	False,
        [],
	["AB"],
	PostBytes(1, 1, 0x5)
],
[	"OffsetEInd",
	"oEI",
	["[", "e", ",", "$ireg", "]"],
	[ ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b10111") ],
	1,
	"INDEX16:$ireg",
	True,
        [],
	["AE"],
	PostBytes(1, 1, 0x7)
],
[	"OffsetFInd",
	"oFI",
	["[", "f", ",", "$ireg", "]"],
	[ ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b11010") ],
	1,
	"INDEX16:$ireg",
	True,
        [],
	["AF"],
	PostBytes(1, 1, 0xA)
],
[	"OffsetDInd",
	"oDI",
	["[", "d", ",", "$ireg", "]"],
	[ ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b11011") ],
	1,
	"INDEX16:$ireg",
	False,
        [],
	["AD"],
	PostBytes(1, 1, 0xB)
],
[	"OffsetWInd",
	"oWI",
	["[", "w", ",", "$ireg", "]"],
	[ ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b11110") ],
	1,
	"INDEX16:$ireg",
	True,
        [],
	["AW"],
	PostBytes(1, 1, 0xE)
],
[	"Inc2Ind",
	"Inc2I",
	["[", ",", "$ireg", "++", "]"],
	[ ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b10001") ],
	1,
	"INDEX16:$ireg",
	False,
        [],
	[],
	PostBytes(1, 1, 0x1)
],
[	"Dec2Ind",
	"Dec2I",
	["[", ",", "--", "$ireg", "]"],
	[ ((15, -1), "0b1"), ((14, 13), "ireg"), ((12, 8), "0b10011") ],
	1,
	"INDEX16:$ireg",
	False,
        [],
	[],
	PostBytes(1, 1, 0x3)
],
[	"Offset8PCInd",
	"o8PCI",
	["[", "$offset", ",", "pc", "]"],
	[ ((23, 16), "offset{7-0}"), ((15, -1), "0b1"), ((14, 13), "0b00"), ((12, 8), "0b11100") ],
	2,
	"pcrel8:$offset",
	False,
        [],
	[],
	PostBytes(1, 1, 0xC)
],
[	"Offset16PCInd",
	"o16PCI",
	["[", "$offset", ",", "pc", "]"],
	[ ((31, 24), "offset{7-0}"), ((23, 16), "offset{15-8}"), ((15, -1), "0b1"), ((14, 13), "0b00"), ((12, 8), "0b11101") ],
	3,
	"pcrel16:$offset",
	False,
        [],
	[],
	PostBytes(1, 1, 0xD)
],
[	"ExtendedInd",
	"eI",
	["[", "$addr", "]"],
	[ ((31, 24), "addr{7-0}"), ((23, 16), "addr{15-8}"), ((15, -1), "0b1"), ((14, 13), "0b00"), ((12, 8), "0b11111") ],
	3,
	"addr16:$addr",
	False,
        [],
	[],
	PostBytes(1, other_opcode=0x1F)
],
[	"NoOffsetWInd",
	"0oWI",
	["[", ",", "w", "]"],
	[ ((15, 8), "0b10010000") ],
	1,
	"ACC16:$ireg",
	True,
        [],
	["AW"],
	PostBytes(1, other_opcode=0x10)
],
[	"Offset16WInd",
	"16oWI",
	["[", "$offset", ",", "w", "]"],
	[ ((31, 24), "offset{7-0}"), ((23, 16), "offset{15-8}"), ((15, 8), "0b10110000") ],
	3,
	"offset16:$offset,ACC16:$ireg",
	True,
        [],
	["AW"],
	PostBytes(1, other_opcode=0x30)
],
[	"IncW2Ind",
	"IncW2I",
	["[", ",", "w", "++", "]"],
	[ ((15, 8), "0b11010000") ],
	1,
	"ACC16:$ireg",
	True,
        ["AW"],
	["AW"],
	PostBytes(1, other_opcode=0x50)
],
[	"DecW2Ind",
	"DecW2I",
	["[", ",", "--", "w", "]"],
	[ ((15, 8), "0b11110000") ],
	1,
	"ACC16:$ireg",
	True,
        ["AW"],
	["AW"],
	PostBytes(1, other_opcode=0x70)
],
]

#define FILSYSTYPE 13

// auto u_int16 OpenFile(register __c0 u_int16 fileNum) {
// auto u_int16 Fat12OpenFile(register __c0 u_int16 fileNum) {

	.sect code,Fat12OpenFile
	.export _Fat12OpenFile
_Fat12OpenFile:
	.import _minifatInfo
	ldc _minifatInfo+FILSYSTYPE,i7
	stx d0,(i6)+1	; sty b0,(i6)
	stx ls,(i6)+1	; sty lc,(i6)
	ldy (i7),d0	; stx le,(i6)
	ldc 0x3231,b0
	sub d0,b0,d0	; sty lr0,(i6)+1
	ldc 0x6eab,le	//directory detected
	jzc $0
	ldc 0xffff,lc

	ldc 0x6f06,ls	//update position etc..
	//ldc 0x6ecb,ls

$0:	.import _FatOpenFile
	call _FatOpenFile
	nop

	ldx (i6)-1,le	; ldy (i6),lr0
	ldx (i6)-1,ls	; ldy (i6),lc
	jr
	ldx (i6)-1,d0	; ldy (i6),b0

	.end

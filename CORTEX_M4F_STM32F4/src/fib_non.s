	.syntax unified
	.text
	.align 2
	.thumb
	.thumb_func

	.global fibonacci_non
	.type fibonacci_non, function

fibonacci_non:
	@ PROLOG
	push {r3, r4, r5, lr}
	@ if(r0 <= 1) return 0
	sub r0, #1
	cmp r0, #1
	mov r5, #0
	blt .L4
	@ if x != 1, 0; need to add til the origin value
	add r0, #1
	@ if(r0 == 2) return 1
	cmp r0, #2
	mov r5, #1
	beq .L4
	mov r3, #0
	mov r4, #1
.L3:
	@ last = first + mid
	add r5, r3, r4
	@ first = mid
	@ mid = last
	mov r3, r4
	mov r4, r5
	sub r0, #1
	cmp r0, #2
	bne .L3

.L4:
	mov r0, r5
	pop {r3, r4, r5, pc}		@ EPILOG

	.size fibonacci_non, .-fibonacci_non
	.end

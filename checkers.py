
#board array
board=[0]*64

#positions in the top row
topRow=[0,1,2,3,4,5,6,7]

#positions in the bottom row
bottomRow=[56,57,58,59,60,61,62,63]

#game is happening
gameOn=True
#0 is an empty space
#1 is a black checker
#2 is a red checker
#3 is a black king
#4 is red king

"""
0  1  2  3  4  5  6  7 
8  9  10 11 12 13 14 15 
16 17 18 19 20 21 22 23 
24 25 26 27 28 29 30 31 
32 33 34 35 36 37 38 39 
40 41 42 43 44 45 46 47 
48 49 50 51 52 53 54 55 
56 57 58 59 60 61 62 63 
"""

def callArm(start, end, remove):
    #start is the checker starting index
    #end is checker ending index
    #remove will be position of checker to be removed if needed
    #otherwise -1
    pass



#get a position input from user
def getPos():
    try:
        return int(input(""))
    except:
        print("Invalid input. Please enter number between 0 and 63")
        return getPos()

#forfeit game
def forfeit(winner):
    global gameOn
    gameOn=False
    if winner == 1:
        print("Player 1 Wins!")
    elif winner == 2:
        print("Player 2 Wins!")

#moving a piece from the top of the board to the bottom of the board
def moveDown(board, start, end, enemy):
    global wasTake
    #black pieces typically
    #check if move is diagonal and not going off end of board
    #makes sure you dont move one space after a take
    if ((end-start==9 and start%8 != 7) or (end-start==7 and start%8!=8)) and not wasTake:
        if board[end]==0:
            board[end]=board[start]
            board[start]=0
            wasTake=False
            #kings black piece
            if enemy==0 and end in bottomRow:
                board[end]=3
            callArm(start, end, -1)
            return True
    #taking
    elif (end-start==18 and start%8 < 6):
        #makes sure enemy space is being jumped and not and empty or friendly space
        #makes sure the landing spot is unoccupied
        if board[start+9]%2==enemy and board[start+9]!=0 and board[end]==0:
            board[end]=board[start]
            board[start+9]=0
            board[start]=0
            wasTake = True
            #kings black piece
            if enemy==0 and end in bottomRow:
                board[end]=3
            callArm(start, end, start+9)
            return True
    elif (end-start==14 and start%8 > 1):
        if board[start+7]%2==enemy and board[start+7]!=0 and board[end]==0:
            board[end]=board[start]
            board[start+7]=0
            board[start]=0
            wasTake = True
            if enemy==0 and end in bottomRow:
                board[end]=3
            callArm(start, end, start+7)
            return True
    print("Invalid")
    return False

def moveUp(board, start, end, enemy):
    global wasTake
    #red pieces typically
    if ((start-end==9 and end%8 != 7) or (start-end==7 and end%8!=8)) and not wasTake:
        if board[end]==0:
            board[end]=board[start]
            board[start]=0
            wasTake = False
            #kings red piece
            if enemy==1 and end in topRow:
                board[end]=4
            callArm(start, end, -1)
            return True
    elif (start-end==18 and end%8 < 6):
        if board[start-9]%2==enemy and board[start-9]!=0 and board[end]==0:
            board[end]=board[start]
            board[start-9]=0
            board[start]=0
            wasTake = True
            #kings red piece
            if enemy==1 and end in topRow:
                board[end]=4
            callArm(start, end, start-9)
            return True
    elif (start-end==14 and end%8 > 1):
        if board[start-7]%2==enemy and board[start-7]!=0 and board[end]==0:
            board[end]=board[start]
            board[start-7]=0
            board[start]=0
            wasTake = True
            #kings red piece
            if enemy==1 and end in topRow:
                board[end]=4
            callArm(start, end, start-7)
            return True
    print("Invalid")
    return False
#determines which move is being made
#0 means enemy piece are black, 1 means enemy pieces are red
def makeMove(board, start, end, player):
    if player==1:
        #black pieces
        if board[start]==1:
            return moveDown(board, start, end, 0)
        elif board[start]==3:
            if end-start>0:
                return moveDown(board, start, end, 0)
            elif end-start<0:
                return moveUp(board, start, end, 0)
        print("Invalid")
        return False
    elif player==2:
        #red pieces
        if board[start]==2:
            return moveUp(board, start, end, 1)
        elif board[start]==4:
            if end-start>0:
                return moveDown(board, start, end, 1)
            elif end-start<0:
                return moveUp(board, start, end, 1)
        print("Invalid")
        return False
    return False

#display the ascii board
def printBoard(board):
    for i in range(len(board)):
        print(board[i], end =" ")
        if i%8==7:
            print("")
#check if all enemy pieces are taken
def isOver(board):
    if 1 not in board and 3 not in board:
        return 2
    elif 2 not in board and 4 not in board:
        return 1
    else:
        return 0


#creates initial setup
def setupBoard(board):
    for i in range(1,8,2):
        board[i]=1
    for i in range(8,16,2):
        board[i]=1
    for i in range(17,24,2):
        board[i]=1
    for i in range(40,48,2):
        board[i]=2
    for i in range(49,56,2):
        board[i]=2
    for i in range(56,64,2):
        board[i]=2
#player 1 turn, did not take prev move
def p1Turn(board):
    global wasTake
    printBoard(board)
    print("Player 1")
    print("Type current piece position or -1 to Forfeit: ")
    start = getPos()

    if start==-1:
        forfeit(2)
        return True
    end = print("Type final piece position: ")
    end = getPos()

    moveValid=makeMove(board, start, end, 1)
    if not moveValid:
        p1Turn(board)
    if wasTake:
        p1Took(board, end)
    return True
#p2 turn, did not take prev move
def p2Turn(board):
    global wasTake
    printBoard(board)
    print("Player 2")
    print("Type current piece position or -1 to Forfeit: ")
    start = getPos()
    if start==-1:
        forfeit(1)
        return True
    end = print("Type final piece position: ")
    end = getPos()
    moveValid=makeMove(board, start, end, 2)
    if not moveValid:
        p2Turn(board)
    return True
#p1 turn, took on prev move
def p1Took(board, startPiece):
    global wasTake
    printBoard(board)
    print("Player 1")
    print("Start piece = "+str(startPiece))
    print("Type final piece position or -1 to end turn: ")
    end = getPos()

    if end == -1:
        wasTake=False
        return True
    moveValid=makeMove(board, startPiece, end, 1)
    if not moveValid:
        p1Took(board, startPiece)
    else:
        p1Took(board, end)
#p2 turn took on prev move
def p2Took(board, startPiece):
    global wasTake
    printBoard(board)
    print("Player 2")
    print("Start piece = "+str(startPiece))
    print("Type final piece position or -1 to end turn: ")
    end = getPos()
    if end == -1:
        wasTake=False
        return True
    moveValid=makeMove(board, startPiece, end,2)
    if not moveValid:
        p2Took(board, startPiece)
    else:
        p2Took(board, end)


playerTurn=1
wasTake=False
setupBoard(board)
#main loop
while isOver(board)==0 and gameOn:
    if playerTurn==1:
        p1Turn(board)
        playerTurn=2
    elif playerTurn==2:
        p2Turn(board)
        playerTurn=1
gameOn=False
if isOver(board)==1:
    print("Player 1 Wins!")
elif isOver(board)==2:
    print("Player 2 Wins!")


from tkinter import *


#  예제 1) tkinter 파이썬 GUI 레이블(label)
# tkinter를 사용하여 텍스트를 나타내보자

"""# 1. 루트화면 (root window) 생성
tk = Tk() 
# 2. 텍스트 표시
label = Label(tk,text='Hello World!') 
# 3. 레이블 배치 실행
label.pack()
# 4. 메인루프 실행
tk.mainloop()"""

# 예제2) 버튼만들기
"""tk = Tk()
# 함수 정의 (버튼을 누르면 텍스트 내용이 바뀜)
def event():
    button['text'] = '버튼 누름!'

button = Button(tk,text='버튼입니다. 누르면 함수가 실행됩니다.',command=event)
button2 = Button(tk,text='버튼2 입니다.')
button.pack(side=LEFT,padx=10,pady=10) #side로 배치설정, padx로 좌우 여백설정, pady로 상하 여백설정 
button2.pack(side=LEFT, padx=10, pady= 10)
#tk.mainloop()

#버튼 텍스트 수정(text속성에 접근하여 수정)
button['text'] = 'Button1'
button2['text'] = 'Button2'                        
tk.mainloop()"""

"""# 예제 3) 버튼 클릭시 실행될 이벤트(함수) 설정
tk = Tk()

# 다른 함수 정의(버튼 누를때마다 카운트를 세는 함수)
counter = 0
def clicked():
    global counter #전역변수 counter
    counter += 1
    label1['text'] = '버튼 클릭 수: ' + str(counter)

# 리셋 함수(카운트 초기화)
def reset():
    global counter
    counter = 0
    label1['text'] = '옆에 버튼이 있습니다.'
## GUI 구성(텍스트,버튼) ##

# 창 이름 설정
tk.title('GUI예제') 

# 텍스트
label1=Label(tk, text='옆에 버튼이 있습니다.',fg='blue',font=20) # fg는 글자 색 지정, font로 글자 설정
label1.pack(side=LEFT, padx=10, pady=10)
# 버튼1
button3 = Button(tk,text='클릭해 보세요.',bg='green',font=15,width=30,height=5,command= clicked) #command로 버튼 클릭 시 동작할 함수 지정, bg로 색상지정, width,height로 각각 넓이 높이 지정
button3.pack(side=LEFT, padx=10, pady=10)
# 버튼2
button4 = Button(tk,text='reset',bg='red',width=30,height=5,font=15,command=reset)
button4.pack(side=LEFT,padx=10, pady=10)
tk.mainloop()
"""


###예제4) ft -> cm로 바꾸는 단위 변환기 만들기
# Entry: input과 비슷한 역할 (사용자가 입력한 내용 전달)
# get: Entry를 사용한 입력 내용 가져올 수 있다.
# delete: 사용자 입력 삭제
# Frame: 컨테이너, 창 안에 프레임 생성
# grid: 격자 배치
# 예제 3) 버튼 클릭시 실행될 이벤트(함수) 설정
tk = Tk()

# 다른 함수 정의(버튼 누를때마다 카운트를 세는 함수)
counter = 0
def clicked():
    global counter #전역변수 counter
    counter += 1
    label1['text'] = '버튼 클릭 수: ' + str(counter)

# 리셋 함수(카운트 초기화)
def reset():
    global counter
    counter = 0
    label1['text'] = 'turtle O_0 turtel.'
## GUI 구성(텍스트,버튼) ##

# 창 이름 설정
tk.title('GUI예제') 

# 텍스트
label1=Label(tk, text='turtle O_0 turtel.',fg='blue',font=20) # fg는 글자 색 지정, font로 글자 설정
label1.pack(side=LEFT, padx=10, pady=10)
# 버튼1
button3 = Button(tk,text='start',bg='green',font=15,width=30,height=5,command= clicked) #command로 버튼 클릭 시 동작할 함수 지정, bg로 색상지정, width,height로 각각 넓이 높이 지정
button3.pack(side=LEFT, padx=10, pady=10)
# 버튼2
button4 = Button(tk,text='stop',bg='red',width=30,height=5,font=15,command=reset)
button4.pack(side=LEFT,padx=10, pady=10)
tk.mainloop()
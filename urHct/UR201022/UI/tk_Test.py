from tkinter import *
from tkinter import messagebox


class MyFrame(Frame):
    def __init__(self,master):
        Frame.__init__(self,master)

        frame1 = Frame(master)
        frame1.grid(row=0,column=0)
        dumyLabel1 = Label(frame1,width = 1)
        dumyLabel2 = Label(frame1,width = 1)
        dumyLabel3 = Label(frame1,width = 1)
        
        reset = Button(frame1,text="reset",command = onClick_reset)
        reset.grid(row=0,column=0)

        voice = Button(frame1,text="voice",command = onClick_reset)
        voice.grid(row=0,column=2)

        front = Button(frame1,text="front",command = onClick_reset)
        front.grid(row=0,column=4)

        temperature = Button(frame1,text="temp",command = onClick_reset)
        temperature.grid(row=0,column=6)
        
        dumyLabel1.grid(row=0,column=1)
        dumyLabel2.grid(row=0,column=3)
        dumyLabel3.grid(row=0,column=5)




def onClick_reset():
    messagebox.showinfo("ganada")

def main():
    root = Tk()
    root.title('UR Robot-U10')
    #root.geometry("700x700+100+100")
    app = MyFrame(root)
    root.mainloop()

if __name__ == '__main__':
    main()
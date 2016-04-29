import wx
import os

class MainWindow(wx.Frame):
    def __init__(self, parent, title):
        self.dirname=''

        # A "-1" in the size parameter instructs wxWidgets to use the default size.
        # In this case, we select 200px width and the default height.
        wx.Frame.__init__(self, parent, title=title, size=(600,300))
        self.control = wx.TextCtrl(self, style=wx.TE_MULTILINE)
        self.control2 = wx.TextCtrl(self, style=wx.TE_MULTILINE)
        self.CreateStatusBar() # A Statusbar in the bottom of the window
        
        # Setting up the menu.

        # Creating the menubar.

        
        self.buttonSizer = wx.BoxSizer(wx.VERTICAL)
        self.button = wx.Button(self, 1, "Query")
        self.buttonSizer.Add(self.button,1,wx.ALIGN_CENTER)
        

        # Use some sizers to see layout options
        self.sizer = wx.GridSizer(2,3,1,wx.HORIZONTAL)
        
        self.sizer.Add(self.control, 1, wx.EXPAND)
        self.sizer.Add(self.control2, 1, wx.EXPAND)
        self.sizer.Add(self.buttonSizer, 0.2, wx.EXPAND)
        
        #Layout sizers
        self.SetSizer(self.sizer)
        self.SetAutoLayout(2)
        self.Show()


if __name__ == '__main__':
    app = wx.App(False)
    frame = MainWindow(None, "RSQueryInterface")
    app.MainLoop()
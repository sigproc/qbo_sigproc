#!/usr/bin/env python
import wx
import os

class MainWindow(wx.Frame):
    def __init__(self, parent, title):
        self.dirname=''

        wx.Frame.__init__(self, parent, title=title, size=(200,-1))

        

        self.control = wx.TextCtrl(self, style=wx.TE_MULTILINE|wx.EXPAND,size=(300, -1))
        self.CreateStatusBar() # A Statusbar in the bottom of the window

        self.sizer2 = wx.BoxSizer(wx.HORIZONTAL)
        self.buttonSave = wx.Button(self, -1, "Save Location")
        self.buttonSend = wx.Button(self, -1, "Send Location")
        self.sizer2.Add(self.buttonSave, 1, wx.EXPAND)
        self.sizer2.Add(self.buttonSend, 1, wx.EXPAND)
        self.Bind(wx.EVT_BUTTON, self.OnClickSave,self.buttonSave)
        self.Bind(wx.EVT_BUTTON, self.OnClickSend,self.buttonSend)

        self.sizer3 = wx.BoxSizer(wx.HORIZONTAL)
        self.buttonCancel = wx.Button(self, -1, "Cancel Nav Goal")
        self.sizer3.Add(self.buttonCancel, 1, wx.EXPAND)
        self.Bind(wx.EVT_BUTTON, self.OnClickCancel,self.buttonCancel)

        # Use some sizers to see layout options
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.sizer2, 0, wx.EXPAND)        
        self.sizer.Add(self.sizer3, 0, wx.EXPAND)     
        self.sizer.Add(self.control, 1, wx.EXPAND)

        #Layout sizers
        self.SetSizer(self.sizer)
        self.SetAutoLayout(1)
        self.sizer.Fit(self)
        self.Show()

        dc = wx.ClientDC(self)
        rect = self.GetUpdateRegion().GetBox()
        dc.SetClippingRect(rect)
        dc.Clear()
        bmp = wx.Bitmap("/home/sifuf/cued-masters/src/qbo_sigproc/qbo_laser_slam/src/roses.jpg")
        dc.DrawBitmap(bmp, 0, 0)

    def OnClickSave(self,e):
        self.control.AppendText("Stored current location! %d\n" %e.GetId())    
    
    def OnClickSend(self,e):    
        self.control.AppendText("Published nav goal! %d\n" %e.GetId())
    
    def OnClickCancel(self,e): 
        self.control.Clear()   
        self.control.AppendText("Canceled nav goal! %d\n" %e.GetId())
       

app = wx.App(False)
frame = MainWindow(None, "Qbo Location Saver")
app.MainLoop()

# -*- coding: utf-8 -*-
"""
Created on Wed Nov 30 12:50:56 2016

@author: ferenc
"""

from pymongo import MongoClient


class MongoClient:
  
  client = MongoClient()
  db = client["Scenes_annotated"]

  def __init__(self,dbName):
    self.client = MongoClient()
    self.db = self.client[dbName]
        
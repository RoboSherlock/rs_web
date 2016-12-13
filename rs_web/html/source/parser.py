# -*- coding: utf-8 -*-
"""
Created on Mon Dec 12 11:19:44 2016

@author: ferenc
"""

from pyparsing import Word, alphas,nums,Literal,oneOf,ZeroOrMore,Forward,Group,alphanums


key = oneOf('location shape color size detection id')
separator = Literal(',').suppress()
lpar = Literal('(').suppress()
rpar = Literal(')').suppress()

prop = lpar + key + separator + Word(alphanums)  + rpar
props = Forward()
atom = prop | Group(props)
props << atom + ZeroOrMore(separator + props)

funcs = oneOf( 'scenes views object' )
expr = funcs + lpar + funcs + lpar + props + rpar+rpar





#def findObjInStoreById(origString,loc,tokens):
#    print 'Searching fro object in ObjStore with ID: '+ tokens[3] 
#    return tokens[3]
#
#obj.setParseAction(findObjInStoreById)

#show(views(object(id,2)))


if __name__ == "__main__":
    
#    s ='views(object(id,2))'
#    results = views.parseString( s ) 
#    print s,'->', results
#
##    s ='object(views(id,2))'    
##    results = views.parseString( s )
##    print s,'->', results
#    
#    s ='(shape,box)'    
#    results = prop.parseString( s )
#    print s,'->', results
    
    s ='object((shape,box),(color, blue),(size, small))'    
#    s ='views(object((shape,box),(color, blue),(size, small)))'    
#    s ='scenes(object((shape,box),(color, blue),(size, small)))'
#        
    
    results = expr.parseString( s )
    print s,'->', results
    

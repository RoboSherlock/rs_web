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
lbr = Literal('[').suppress()
rbr = Literal(']').suppress()

And = Literal('and').suppress()

kvp = lpar + Group(key + separator + Word(alphanums))+rpar
kvps = Forward()
atom = kvp | Group(kvps)
kvps << atom + ZeroOrMore(separator + kvps)

objDef = lpar + Group('object' +lpar + kvps +rpar) +rpar
objs = Forward()
elem = objDef | Group(objs)
objs << elem + ZeroOrMore(And+objs)

scene = 'scenes' + lpar + objs + rpar
views = 'views' + lpar + objs + rpar


#def findObjInStoreById(origString,loc,tokens):
#    print 'Searching fro object in ObjStore with ID: '+ tokens[3] 
#    return tokens[3]
#
#obj.setParseAction(findObjInStoreById)

#show(views(object(id,2)))


if __name__ == "__main__":  
    
    s ='(object((shape,box),(color, blue)))'
    results = objDef.parseString( s )
    print s,'->', results
    
    s ='scenes((object((shape,box))) '+ \
                'and (object((id,2))) '+\
                'and (object((id,3))))'
    results = scene.parseString( s )
    print s,'->', results
    
    s ='views((object((shape,box))) and (object((id,2))) and (object((id,3))))'
    results = views.parseString( s )
    print s,'->', results
    

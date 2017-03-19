# -*- coding: utf-8 -*-
"""
Created on Mon Dec 12 11:19:44 2016

@author: ferenc
"""

from pyparsing import *
from mongoclient import RSMongoClient


class QueryHandler(object):

    def __init__(self):
        print "Init Query Handler"
        """
           @type self.mc: RSMongoClient
        """
        self.mc = RSMongoClient("Scenes_annotated")
        self.status = 'begin'

    def kvp_cb_(self, t):
        print "KVP %s" % t

    def res_specif_cb_(self, t,mc=None):
        if mc is None:
            mc = self.mc

        """
           @type self.mc: RSMongoClient
        """
        print "Result Specifier: %s." % t
        if t[0] == 'object' and self.status =='begin':
            print 'final result should be collection of objects matching description'
        if t[0] == 'scene' and self.status =='begin':
            print 'final result should be collection of scenes matching description'
        mc.get_persistent_objects()


    def operator_cb_(self, t):
        print "Operator call-back: %s" % t

    def end_cb_(self, t):
        print "Query %s" % t
        print 'End of parsing'
        self.status = 'begin'


class RSQueryGrammar:
    def __init__(self):
        self.qh = QueryHandler()
        bq = Literal('{').suppress()
        eq = Literal('}').suppress()
        bl = Literal('[').suppress()
        el = Literal(']').suppress()
        bp = Literal('(').suppress()
        ep = Literal(')').suppress()

        delimiter = oneOf(':').suppress()
        operator = oneOf('= < >')
        separator = oneOf(', ;').suppress()

        resultSpecifier = oneOf('scene hypotheses object')
        key = oneOf('location shape color size detection id ts type value ratio confidence') ^ resultSpecifier

        point = Literal('.')
        number = Word(nums)
        plus_or_minus = Literal('+') | Literal('-')
        integer = Combine(Optional(plus_or_minus) + number)
        floatnumber = Combine(integer +
                              Optional(point + Optional(number))
                              )
        kvps = Forward()
        resultDescription = bl + kvps + el
        description = bp + kvps + ep

        value = Word(alphanums) | description

        # e.g. simple symbolic assignment
        simpleKvp = key + delimiter + value

        # numerical comparison confidence > 0.0
        valueKvp = key + operator + (floatnumber | value)
        nestedKvP = key + delimiter + resultDescription

        # kvp =  simpleKvp | nestedKvP | valueKvp #matchfirst sux...use or instead..which is end...FFS
        kvp = simpleKvp ^ nestedKvP ^ valueKvp
        kvps << ZeroOrMore(kvp + Optional(separator))

        query = bq + resultSpecifier + delimiter + resultDescription + eq

        resultSpecifier.addParseAction(self.qh.res_specif_cb_)
        simpleKvp.addParseAction(self.qh.kvp_cb_)
        valueKvp.addParseAction(self.qh.operator_cb_)
        query.addParseAction(self.qh.end_cb_)
        self.query = query

    def parseQuery(self, q):
        print 'Parsing ', q
        res = self.query.parseString(q)
        return res


if __name__ == "__main__":
        gr = RSQueryGrammar()
    # try:
        s = '{object:[shape:blue, size:small]}'
        gr.parseQuery(s)

        s = '{object:[]}'
        gr.parseQuery(s)

        s = '{scene:[hypotheses:[type:Plate,shape:(value:blue, confidence=0.5)],hypotheses:[color:blue, shape:box, ' \
            'type:crap]]} '
        gr.parseQuery(s)

    # except (ParseException):
    #     print('Malformed query. RTFM')
    # except Exception:
    #     print 'Something went horribly wrong!'
#

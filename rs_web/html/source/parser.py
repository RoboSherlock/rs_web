# -*- coding: utf-8 -*-
"""
Created on Mon Dec 12 11:19:44 2016

@author: ferenc
"""

from pyparsing import *
from mongoclient import RSMongoClient

"""
    @type mc: RSMongoClient
"""
mc = RSMongoClient("Scenes_annotated")

class QueryHandler(object):

    def __init__(self):
        print "Init Query Handler"
        self.status = 0

    def kvp_cb_(self, t):
        print "KVP %s" % t

    def res_specif_cb_(self, t):
        print "Result Specifier: %s." % t
        if self.status == 0:
            if t[0] == 'object':
                print 'final result should be collection of objects matching description'
            elif t[0] == 'scene':
                print 'final result should be collection of scenes matching description'
            elif t[0] == 'hypotheses':
                print 'final result should be collection of obj-hypotheses matching description'
            self.status = 1
        a = mc.get_cursor(t[0])#some parametrization that gives all views all objects or all scenes?


    def operator_cb_(self, t):
        print "Operator call-back: %s" % t

    def end_cb_(self, t):
        print "Query %s" % t
        print 'End of parsing'
        self.status = 0


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

        result_specifier = oneOf('scene hypotheses object')
        key = oneOf('location shape color size detection id ts type value ratio confidence') ^ result_specifier

        point = Literal('.')
        number = Word(nums)
        plus_or_minus = Literal('+') | Literal('-')
        integer = Combine(Optional(plus_or_minus) + number)
        floatnumber = Combine(integer +
                              Optional(point + Optional(number))
                              )
        kvps = Forward()
        result_description = bl + kvps + el
        description = bp + kvps + ep

        value = Word(alphanums) | description

        # e.g. simple symbolic assignment
        simpleKvp = key + delimiter + value

        # numerical comparison confidence > 0.0
        valueKvp = key + operator + (floatnumber | value)
        nestedKvP = key + delimiter + result_description

        # kvp =  simpleKvp | nestedKvP | valueKvp #matchfirst sux...use or instead..which is end...FFS
        kvp = simpleKvp ^ nestedKvP ^ valueKvp
        kvps << ZeroOrMore(kvp + Optional(separator))

        query = bq + result_specifier + delimiter + result_description + eq

        result_specifier.addParseAction(self.qh.res_specif_cb_)
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

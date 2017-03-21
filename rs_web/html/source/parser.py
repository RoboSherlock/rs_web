# -*- coding: utf-8 -*-
"""
Created on Mon Dec 12 11:19:44 2016

@author: ferenc
"""

from pyparsing import *
from mongoclient import RSMongoClient

"""
    @type mc: mongoclient.RSMongoClient
"""
mc = RSMongoClient("Scenes_annotated")


dict = {"shape" :   {"_type":"rs.annotation.Shape"},
        "size"  :   {"_type":"rs.annotation.SemanticSize"},
        "color" :   {"_type":"rs.annotation.SemanticColor"},
        "value" :   "value" ,
        "id"    :   "_id"}

print dict['shape']
# db.getCollection('persistent_objects').find( {
# $and: [ { annotations: { $elemMatch: { distanceToPlane: {$gt:0.03}}}},
# { annotations: { $elemMatch: { confidence: {$lt:0.8},source: "DeCafClassifier"}}}
#            ]
# ;})

res = mc.db.persistent_objects.find({'$and' : [ {"annotations" : {'$elemMatch': {'distanceToPlane': {'$gt':0.03}}}},
                                                {"annotations" : {'$elemMatch': {'confidence':{'$lt':0.8},'source':'DeCafClassifier'}}},
                                                {"annotations" : {'$elemMatch': {'_type':'rs.annotation.Shape','shape':'box'}}}]})

new_dict={}
new_dict['$elemMatch'] = dict

print dict

print new_dict

if res.count() != 0:
    print 'found results...OMG..count: %d' %res.count()

op_dict = {'<':'$lt',
           '>':'$gt',
           '=':'$eq'}

class QueryHandler(object):

    def __init__(self):
        print "Init Query Handler"
        self.status = 0
        self.collection = None

    def kvp_cb_(self, t):
        if(len(t) == 2):
            print '(%d) kvp in : %s' %(self.status,t)
            print '(%d) %s' % (self.status,dict[t.key])
            res= {t.key:t.value}
            print '(%d) kvp out: %s' %(self.status, res)
            self.status += 1

    def description_cb_(self,t):
        print t

    def res_specif_cb_(self, t):
        # print "Result Specifier: %s." % t
        print  'getting res_spec %s ' %t.result_specifier
        if self.status == 0:
            if t.result_specifier == 'object':
                print '(%d) final result should be collection of objects matching description' %self.status
            elif t.result_specifier == 'scene':
                print '(%d) final result should be collection of scenes matching description' %self.status
            elif t.result_specifier == 'hypotheses':
                print '(%d) final result should be collection of obj-hypotheses matching description'
            self.status += 1
        elif self.status > 0:
            print '(%d) This is a nested query' % self.status
            self.status += 1
        mc.set_main_collection(t.result_specifier)#some parametrization that gives all views all objects or all scenes?


    def operator_cb_(self, t):
        print '(%d) constraint: Key: %s' %(self.status,t.key)
        print '(%d) constraint: Operation mapped to mongo: %s' %(self.status,op_dict[t.op])
        print '(%d) constraint: Value: %s' %(self.status,t.value)
        res = {t.key : {op_dict[t.op]:t.value}}
        print '(%d) constraint: %s,' %(self.status, res)
        self.status +=1


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

        result_specifier = oneOf('scene hypotheses object')("result_specifier")
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
        description = Group(bp + kvps + ep)

        value = Word(alphanums)

        # e.g. simple symbolic assignment
        simpleKvp = key("key") + delimiter + value("value")

        # numerical comparison confidence > 0.0
        valueKvp = key("key") + operator("op") + (floatnumber | value)("value")
        nestedKvP = key + delimiter + result_description
        descriptionKvP = key("key") + delimiter + description

        # kvp =  simpleKvp | nestedKvP | valueKvp #matchfirst sux...use or instead..which is end...FFS
        kvp = simpleKvp ^ valueKvp ^ descriptionKvP ^ nestedKvP
        kvps << ZeroOrMore(kvp + Optional(separator))

        query = bq + result_specifier + delimiter + result_description + eq

        result_specifier.addParseAction(self.qh.res_specif_cb_)
        simpleKvp.addParseAction(self.qh.kvp_cb_)
        valueKvp.addParseAction(self.qh.operator_cb_)
        descriptionKvP.addParseAction(self.qh.description_cb_)
        query.addParseAction(self.qh.end_cb_)
        self.query = query

    def parseQuery(self, q):
        print 'Parsing ', q
        res = self.query.parseString(q)
        return res


if __name__ == "__main__":
        gr = RSQueryGrammar()
    # try:

        s = '{object:[color:blue, shape:(value:box,confidence>0.6), size:small]}'
        gr.parseQuery(s)

        # s = '{object:[]}'
        # gr.parseQuery(s)
        #
        # s = '{scene:[hypotheses:[type:Plate,shape:(value:blue, confidence=0.5)],hypotheses:[color:blue, shape:box, ' \
        #     'type:crap]]} '
        # gr.parseQuery(s)

        s = '{hypotheses:[ id:6, detection:(confidence>0.05),]}'
        gr.parseQuery(s)

    # except (ParseException):
    #     print('Malformed query. RTFM')
    # except Exception:
    #     print 'Something went horribly wrong!'
#

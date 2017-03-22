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


key_dict = {"shape":        {"_type": "rs.annotation.Shape"},
            "size":         {"_type": "rs.annotation.SemanticSize"},
            "color":        {"_type": "rs.annotation.SemanticColor"}
            }

"""
res = mc.db.persistent_objects.find({'$and' : [ {"annotations": {'$elemMatch': {'distanceToPlane': {'$gt':0.03}}}},
                                                {"annotations": {'$elemMatch': {'confidence':{'$lt':0.8},'source':'DeCafClassifier'}}},
                                                {"annotations": {'$elemMatch': {'_type':'rs.annotation.Shape','shape':'box'}}}]})
if res.count() != 0:
    print 'found results...OMG..count: %d' %res.count()
"""


op_dict = {'<': '$lt',
           '>': '$gt',
           '=': '$eq'}


class QueryHandler(object):

    def __init__(self):
        print "Init Query Handler"
        # initialize the grammar
        self.grammar = RSQueryGrammar(self)
        self.status = 0
        # track the no of kvps extracted so we can format mongo query adequately
        self.kvp_counter=0
        self.query = {}
        self.kvp_map = {}

    def reset(self):
        """
        reset all variables used to track the status of one query
        """
        self.status = 0
        self.kvp_counter = 0
        self.query = {}
        self.kvp_map = {}

    @property
    def gen_dict(self):
        """

        :return: the specific nesting of a dic needed for a mongo query
        """
        dict={}
        dict['annotations'] = {}
        dict['annotations']['$elemMatch'] = {}
        return dict

    def exec_query(self,q):
        """

        :param q: the query string
        """
        self.reset()
        self.grammar.parse_query(q)
        cursor = mc.call_query(self.query)
        return mc.process_objects_cursor(cursor)

    def kvp_cb_(self, t):
        if len(t) == 2:
            print '(%d) kvp in : %s' %(self.kvp_counter,t)
            print '(%d) %s' % (self.kvp_counter,t.key)
            res= {t.key:t.value}
            print '(%d) kvp out: %s' %(self.kvp_counter, res)

            element=self.gen_dict
            element['annotations']['$elemMatch']=res
            self.query['$and'].append(element)
            self.kvp_map[self.kvp_counter] = element
            self.kvp_counter += 1
            print self.query
        else:
            print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
            print 'Now what..this mean that everything inside was already processed'
            print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'

    def operator_cb_(self, t):
        print '(%d) constraint: Key in: %s' % (self.kvp_counter, t.key)
        print '(%d) constraint: Value: %s' % (self.kvp_counter, t.value)
        res = {t.key: {op_dict[t.op]: float(t.value)}}
        print '(%d) constraint: Key out: %s,' % (self.kvp_counter, res)

        element = self.gen_dict
        element['annotations']['$elemMatch'] = res

        self.query['$and'].append(element)
        self.kvp_map[self.kvp_counter] = element
        self.kvp_counter += 1

    def description_cb_(self,t):
        print 'Key: %s No. Of Elements: %d' %(t.key ,len(t.description))
        # order of parsing workaround...there might be nicer ways of doing this
        # delete the las x elements and re add them under the same $elemMatch
        del self.query['$and'][-len(t.description):]
        print 'DELETED the nested ones'
        element=self.gen_dict

        for i in range(len(t.description)):
            # print self.kvp_map[self.kvp_counter - i - 1]['annotations']['$elemMatch'].items()
            idx=self.kvp_counter - i - 1
            element["annotations"]["$elemMatch"].update(self.kvp_map[idx]['annotations']['$elemMatch'])
        element["annotations"]["$elemMatch"].update(key_dict[t.key])

        self.query['$and'].append(element)

    def res_specif_cb_(self, t):
        # print "Result Specifier: %s." % t
        print  'getting res_spec %s ' %t.result_specifier
        if self.status == 0:
            if t.result_specifier == 'object':
                print '(%d) final result should be collection of objects matching description' % self.status
                self.query['$and'] = []
            elif t.result_specifier == 'scene':
                print '(%d) final result should be collection of scenes matching description' %self.status
            elif t.result_specifier == 'hypotheses':
                print '(%d) final result should be collection of obj-hypotheses matching description'
            self.status += 1
        elif self.status > 0:
            print '(%d) This is a nested query' % self.status
            self.status += 1
        print self.query
        mc.set_main_collection(t.result_specifier)#some parametrization that gives all views all objects or all scenes?

    def end_cb_(self, t):
        print "Query %s" % t
        print 'End of parsing'
        print self.query
        # mc.call_query(self.query)
        # self.reset()



class RSQueryGrammar:
    def __init__(self, qh):
        print "Initializing Grammar"
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
        key = oneOf('location shape color size detection id ts type value confidence') ^ result_specifier

        point = Literal('.')
        number = Word(nums)
        plus_or_minus = Literal('+') | Literal('-')
        integer = Combine(Optional(plus_or_minus) + number)
        floatnumber = Combine(integer +
                              Optional(point + Optional(number))
                              )
        kvps = Forward()
        result_description = bl + kvps + el
        description = Group(bp + kvps + ep)("description")

        value = Word(alphanums)

        # e.g. simple symbolic assignment
        simpleKvp = key("key") + delimiter + value("value")

        # numerical comparison confidence > 0.0
        valueKvp = key("key") + operator("op") + (floatnumber | value)("value")
        nestedKvP = key + delimiter + result_description
        descriptionKvP = key("key") + delimiter + description

        # kvp =  simpleKvp | nestedKvP | valueKvp #matchfirst sux...use or instead..which is end...FFS
        kvp = descriptionKvP | simpleKvp ^ valueKvp ^ nestedKvP
        kvps << ZeroOrMore(Group(kvp + Optional(separator)))

        query = bq + result_specifier + delimiter + result_description + eq

        floatnumber.addParseAction( lambda s,l,t: [ float(t[0]) ] )
        result_specifier.addParseAction(qh.res_specif_cb_)
        simpleKvp.addParseAction(qh.kvp_cb_)
        valueKvp.addParseAction(qh.operator_cb_)
        descriptionKvP.addParseAction(qh.description_cb_)
        query.addParseAction(qh.end_cb_)
        self.query = query

    def parse_query(self, q):
        if q!=None:
            print 'Parsing ', q
            res = self.query.parseString(q)
            return res


if __name__ == "__main__":
        gr = QueryHandler()
    # try:

        # s = '{object:[color:(value:blue, ratio>0.2), shape:(value:box,confidence>0.6), size:small]}'
        s = '{object:[shape:(shape:round),size:(size:medium,confidence<0.5)]}'
        gr.parse_query(s)
        #
        # s = '{object:[]}'
        # gr.parseQuery(s)
        #
        # s = '{scene:[hypotheses:[type:Plate,shape:(value:blue, confidence=0.5)],hypotheses:[color:blue, shape:box, ' \
        #     'type:crap]]} '
        # gr.parseQuery(s)

        # s = '{hypotheses:[ id:6, detection:(confidence>0.05),]}'
        # gr.parseQuery(s)

    # except (ParseException):
    #     print('Malformed query. RTFM')
    # except Exception:
    #     print 'Something went horribly wrong!'
#

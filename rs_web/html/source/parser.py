# -*- coding: utf-8 -*-
"""
Created on Mon Dec 12 11:19:44 2016

@author: ferenc
"""

from pyparsing import *
from mongoclient import MongoWrapper

"""
    @type mc: mongoclient.RSMongoClient
"""
mc = MongoWrapper()

key_dict = {"shape":        {"_type": "rs.annotation.Shape"},
            "size":         {"_type": "rs.annotation.SemanticSize"},
            "color":        {"_type": "rs.annotation.SemanticColor"},
            "detection":    {"_type": "rs.annotation.Detection"}
            }

"""
res = mc.db.persistent_objects.find({'$and' : [ {"annotations": {'$elemMatch':
                                                    {'distanceToPlane': {'$gt':0.03}}}},
                                                {"annotations": {'$elemMatch':
                                                    {'confidence':{'$lt':0.8},'source':'DeCafClassifier'}}},
                                                {"annotations": {'$elemMatch':
                                                    {'_type':'rs.annotation.Shape','shape':'box'}}}]})
if res.count() != 0:
    print 'found results...OMG..count: %d' %res.count()
"""


op_dict = {'<': '$lt',
           '>': '$gt',
           '=': '$eq'}

aDict = {'object':'annotations',
         'hypotheses':'identifiables.annotations'}

class QueryHandler(object):

    def __init__(self):
        print "Init Query Handler"
        # initialize the grammar
        self.grammar = RSQueryGrammar(self)
        self.status = 0
        # track the no of kvps extracted so we can format mongo query adequately
        self.query_type = ""
        self.kvp_counter = 0
        self.query = {} # rename
        self.final_query ={} # we will use this to concatenate compositions ($and's)
        self.kvp_map = {}

    def reset(self):
        """
        reset all variables used to track the status of one query
        """
        self.status = 0
        self.kvp_counter = 0
        self.query = {}
        self.final_query = {}
        self.kvp_map = {}

    @property
    def gen_dict(self, query_type="object"):
        """

        :return: the specific nesting of a dic needed for a mongo query
        """
        dict = {}
        if query_type == "object":
            dict = {aDict[self.query_type]: {'$elemMatch':{}}}
        # elif query_type == "hypotheses"
        #     dict = {'identifiables.annotations': {'$elemMatch': {}}}
        # dict['annotations'] = {}
        # dict['annotations']['$elemMatch'] = {}
        return dict


    def exec_query(self,q,hack=None):
        """

        :param q: the query string
        """
        if hack == 1:
            mc.set_main_collection('hypotheses')
            cursor = mc.call_query([{'$project': {'_parent': 1, 'identifiables': 1, '_id': 0}},
                                    {'$unwind': '$identifiables'}, {'$match': {'$and':
                                        [{'identifiables.annotations':
                                              {'$elemMatch':{'source': 'DeCafClassifier', 'confidence': {'$gt': 0.5}, '_type': 'rs.annotation.Detection','name':{'$in':['fork_red_plastic','fork_blue_plastic','knife_red_plastic','knife_blue_plastic']}}}}]}}])
            return mc.process_objects_cursor(cursor)
        elif hack == 2:
            mc.set_main_collection('hypotheses')
            cursor = mc.call_query([{'$match':{'$and':[{'timestamp': {'$gt':1482401694215166627}},
                                                        {'timestamp': {'$lt':1482401807402294324}}]}},
                                    {'$project': {'_parent': 1, 'identifiables': 1, '_id': 0}},
                                    {'$unwind': '$identifiables'},
                                    {'$match': {'$and':[{'identifiables.annotations':
                                                                                         {'$elemMatch': {
                                                                                             'source': 'DeCafClassifier',
                                                                                             'confidence': {'$gt': 0.5},
                                                                                             '_type': 'rs.annotation.Detection'}}}]}}])
            return mc.process_objects_cursor(cursor)
        self.reset()
        self.grammar.parse_query(q)
        # if self.query['$and'] == []:
        print 'Query after parsing : %s ' % self.query
        if '$and' in self.query and self.query['$and']==[]:
            print 'Empty query, resetting'
            self.query = {}
            self.final_query = []
        if self.query_type == "hypotheses":
            self.final_query = [{'$project':{'_id':0,'identifiables':1,'_parent':1}},{'$unwind':'$identifiables'},{'$match': self.query}]
            print 'Asked for a hyp: extending query to look in the identifiables of scenes'
        elif self.query_type =="object" and self.query != {}:
            self.final_query = [ {'$match':self.query} ]
            print 'Asked for an obj. Not doins anything'
        print 'MongoQuery: %s ' % self.final_query
        cursor = mc.call_query(self.final_query)
        return mc.process_objects_cursor(cursor)

    def kvp_cb_(self, t):
        if len(t) == 2:
            print '(%d) kvp in : %s' %(self.kvp_counter, t)
            print '(%d) %s' % (self.kvp_counter, t.key)
            res = {t.key: t.value}
            print '(%d) kvp out: %s' %(self.kvp_counter, res)

            element = self.gen_dict
            element[aDict[self.query_type]]['$elemMatch'] = res
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
        element[aDict[self.query_type]]['$elemMatch'] = res

        self.query['$and'].append(element)
        self.kvp_map[self.kvp_counter] = element
        self.kvp_counter += 1

    def description_cb_(self,t):
        print 'Key: %s No. Of Elements: %d' %(t.key ,len(t.description))
        # order of parsing workaround...there might be nicer ways of doing this
        # delete the last x elements and re add them under the same $elemMatch
        del self.query['$and'][-len(t.description):]
        print 'DELETED the nested ones'
        element=self.gen_dict

        for i in range(len(t.description)):
            # print self.kvp_map[self.kvp_counter - i - 1]['annotations']['$elemMatch'].items()
            idx=self.kvp_counter - i - 1
            element[aDict[self.query_type]]["$elemMatch"].update(self.kvp_map[idx][aDict[self.query_type]]['$elemMatch'])
        element[aDict[self.query_type]]["$elemMatch"].update(key_dict[t.key])

        self.query['$and'].append(element)

    def res_specif_cb_(self, t):
        # print "Result Specifier: %s." % t
        print  'getting res_spec %s ' %t.query_specifier
        if self.status == 0:
            if t.query_specifier == 'object':
                self.query['$and'] = []
                self.query_type = "object"
            elif t.query_specifier == 'scene':
                self.query_type = "scene"
            elif t.query_specifier == 'hypotheses':
                self.query['$and'] = []
                self.query_type = "hypotheses"
            self.status += 1
        elif self.status > 0:
            print '(%d) This is a nested query' % self.status
            self.status += 1
        print self.query
        mc.set_main_collection(self.query_type)  # some that gives all views all objects or all scenes?

    def end_cb_(self, t):
        print "Query %s" % t
        print 'End of parsing'


class RSQueryGrammar:


    def __init__(self,qh=None):
        print "Initializing Grammar"

        bl = Literal('[').suppress()
        el = Literal(']').suppress()
        bp = Literal('(').suppress()
        ep = Literal(')').suppress()
        dot = Literal('.').suppress()
        comma = Literal(',').suppress()
        variable = Word(alphas.upper(), alphas, asKeyword=True)

        delimiter = oneOf(':').suppress()
        operator = oneOf('= < >')
        separator = oneOf(', ;').suppress()

        query_specifier = oneOf('scene object hypotheses objectInScene')("query_specifier")
        key = oneOf('source objectID location shape color size detection id ts type value confidence distance value name') ^ query_specifier

        point = Literal('.')
        number = Word(nums)
        plus_or_minus = Literal('+') | Literal('-')
        integer = Combine(Optional(plus_or_minus) + number)
        floatnumber = Combine(integer +
                              Optional(point + Optional(number))
                              )
        kvps = Forward()
        result_description = Group(bl + kvps + el)("description")

        value = Word(alphanums+'_')

        # e.g. simple symbolic assignment
        simpleKvp = key("key") + delimiter + value("value")

        # numerical comparison confidence > 0.0
        valueKvp = key("key") + operator("op") + (floatnumber | value)("value")
        nested_kvp = key("key") + delimiter + result_description

        # kvp =  simpleKvp | nestedKvP | valueKvp #matchfirst sux...use or instead..which is end...FFS
        kvp = nested_kvp | simpleKvp ^ valueKvp
        kvps << ZeroOrMore(Group(kvp + Optional(separator)))

        # query = bq + query_specifier + delimiter + result_description + eq
        query = query_specifier + bp + variable + comma + \
                (variable^result_description)+ Optional(comma +variable) +  ep

        expression = Forward()
        expression << OneOrMore(query+Optional(comma))+dot

        if qh != None:
            floatnumber.addParseAction( lambda s, l, t: [ float(t[0]) ] )
            query_specifier.addParseAction(qh.res_specif_cb_)
            simpleKvp.addParseAction(qh.kvp_cb_)
            valueKvp.addParseAction(qh.operator_cb_)
            nested_kvp.addParseAction(qh.description_cb_)
            query.addParseAction(qh.end_cb_)

        self.query = query
        self.expression =expression

    def exec_rule(self,r):
        if r!=None:
            print 'executing Rule: '
            res = self.expression.parseString(r)
            return res

    def parse_query(self, q):
        if q != None:
            print 'Parsing ', q
            res = self.query.parseString(q)
            return res


if __name__ == "__main__":

        qh = QueryHandler()
        s = 'hypotheses(Hyp, [ detection:[confidence>0.5, source:DeCafClassifier], size:[size:medium, confidence < 0.5]]).'
        qh.exec_query(s)

        # str = 'object(Object,[shape:[shape:round],size:[size:medium,confidence<0.5]]).'
        # qh.exec_query(str)

        # str = 'object(Object,[]).'
        # qh.exec_query(str)
              # 'scene(Ss,[distance:[value>0.15,value<0.30]]),' \
              # 'objectInScene(Object,Scene, Res).'
        # print  gr.exec_rule(str)

        # str = 'objectHypotheses(Obj, [color:[value:blue, ratio>0.2]]).' #, shape:[value:box,confidence>0.6], size:small]).'
        # qh.exec_query(str)
        #
        # qh.exec_query(s)
        # s = 'objectHypotheses(Obj,[]).'

        # s = '{hypotheses:[ id:6, detection:(confidence>0.05),]}'
        # gr.parseQuery(s)

    # except (ParseException):
    #     print('Malformed query. RTFM')
    # except Exception:
    #     print 'Something went horribly wrong!'
#

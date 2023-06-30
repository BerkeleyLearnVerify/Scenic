# Generated from WBT.g4 by ANTLR 4.11.1
# encoding: utf-8
from io import StringIO
import sys

from antlr4 import *

if sys.version_info[1] > 5:
    from typing import TextIO
else:
    from typing.io import TextIO

def serializedATN():
    return [
        4,1,15,111,2,0,7,0,2,1,7,1,2,2,7,2,2,3,7,3,2,4,7,4,2,5,7,5,2,6,7,
        6,2,7,7,7,2,8,7,8,2,9,7,9,2,10,7,10,1,0,1,0,5,0,25,8,0,10,0,12,0,
        28,9,0,1,1,1,1,1,1,1,1,1,2,1,2,1,2,1,2,1,2,1,2,3,2,40,8,2,1,3,1,
        3,1,3,5,3,45,8,3,10,3,12,3,48,9,3,1,4,1,4,1,4,1,4,1,4,3,4,55,8,4,
        1,5,1,5,1,5,1,5,1,5,3,5,62,8,5,1,6,4,6,65,8,6,11,6,12,6,66,1,7,1,
        7,1,8,1,8,1,8,5,8,74,8,8,10,8,12,8,77,9,8,1,8,3,8,80,8,8,1,8,1,8,
        1,8,1,8,1,8,4,8,87,8,8,11,8,12,8,88,1,8,3,8,92,8,8,1,8,1,8,3,8,96,
        8,8,1,9,3,9,99,8,9,1,9,1,9,3,9,103,8,9,4,9,105,8,9,11,9,12,9,106,
        1,10,1,10,1,10,0,0,11,0,2,4,6,8,10,12,14,16,18,20,0,1,1,0,8,9,117,
        0,26,1,0,0,0,2,29,1,0,0,0,4,33,1,0,0,0,6,46,1,0,0,0,8,54,1,0,0,0,
        10,61,1,0,0,0,12,64,1,0,0,0,14,68,1,0,0,0,16,95,1,0,0,0,18,104,1,
        0,0,0,20,108,1,0,0,0,22,25,3,4,2,0,23,25,3,2,1,0,24,22,1,0,0,0,24,
        23,1,0,0,0,25,28,1,0,0,0,26,24,1,0,0,0,26,27,1,0,0,0,27,1,1,0,0,
        0,28,26,1,0,0,0,29,30,5,1,0,0,30,31,5,12,0,0,31,32,3,4,2,0,32,3,
        1,0,0,0,33,34,5,12,0,0,34,35,5,2,0,0,35,36,5,15,0,0,36,37,3,6,3,
        0,37,39,5,3,0,0,38,40,5,15,0,0,39,38,1,0,0,0,39,40,1,0,0,0,40,5,
        1,0,0,0,41,42,3,8,4,0,42,43,5,15,0,0,43,45,1,0,0,0,44,41,1,0,0,0,
        45,48,1,0,0,0,46,44,1,0,0,0,46,47,1,0,0,0,47,7,1,0,0,0,48,46,1,0,
        0,0,49,50,5,4,0,0,50,51,5,12,0,0,51,55,3,10,5,0,52,53,5,12,0,0,53,
        55,3,10,5,0,54,49,1,0,0,0,54,52,1,0,0,0,55,9,1,0,0,0,56,62,3,12,
        6,0,57,62,3,14,7,0,58,62,3,16,8,0,59,62,3,4,2,0,60,62,3,20,10,0,
        61,56,1,0,0,0,61,57,1,0,0,0,61,58,1,0,0,0,61,59,1,0,0,0,61,60,1,
        0,0,0,62,11,1,0,0,0,63,65,5,13,0,0,64,63,1,0,0,0,65,66,1,0,0,0,66,
        64,1,0,0,0,66,67,1,0,0,0,67,13,1,0,0,0,68,69,5,14,0,0,69,15,1,0,
        0,0,70,75,5,5,0,0,71,72,5,15,0,0,72,74,3,10,5,0,73,71,1,0,0,0,74,
        77,1,0,0,0,75,73,1,0,0,0,75,76,1,0,0,0,76,79,1,0,0,0,77,75,1,0,0,
        0,78,80,5,15,0,0,79,78,1,0,0,0,79,80,1,0,0,0,80,81,1,0,0,0,81,96,
        5,6,0,0,82,83,5,5,0,0,83,86,3,18,9,0,84,85,5,7,0,0,85,87,3,18,9,
        0,86,84,1,0,0,0,87,88,1,0,0,0,88,86,1,0,0,0,88,89,1,0,0,0,89,91,
        1,0,0,0,90,92,5,7,0,0,91,90,1,0,0,0,91,92,1,0,0,0,92,93,1,0,0,0,
        93,94,5,6,0,0,94,96,1,0,0,0,95,70,1,0,0,0,95,82,1,0,0,0,96,17,1,
        0,0,0,97,99,5,15,0,0,98,97,1,0,0,0,98,99,1,0,0,0,99,100,1,0,0,0,
        100,102,5,13,0,0,101,103,5,15,0,0,102,101,1,0,0,0,102,103,1,0,0,
        0,103,105,1,0,0,0,104,98,1,0,0,0,105,106,1,0,0,0,106,104,1,0,0,0,
        106,107,1,0,0,0,107,19,1,0,0,0,108,109,7,0,0,0,109,21,1,0,0,0,15,
        24,26,39,46,54,61,66,75,79,88,91,95,98,102,106
    ]

class WBTParser ( Parser ):

    grammarFileName = "WBT.g4"

    atn = ATNDeserializer().deserialize(serializedATN())

    decisionsToDFA = [ DFA(ds, i) for i, ds in enumerate(atn.decisionToState) ]

    sharedContextCache = PredictionContextCache()

    literalNames = [ "<INVALID>", "'DEF'", "' {'", "'}'", "'hidden'", "'['", 
                     "']'", "','", "'TRUE'", "'FALSE'" ]

    symbolicNames = [ "<INVALID>", "<INVALID>", "<INVALID>", "<INVALID>", 
                      "<INVALID>", "<INVALID>", "<INVALID>", "<INVALID>", 
                      "<INVALID>", "<INVALID>", "Comment", "Whitespace", 
                      "Identifier", "Number", "String", "Newline" ]

    RULE_world = 0
    RULE_defn = 1
    RULE_node = 2
    RULE_nodeBody = 3
    RULE_attribute = 4
    RULE_value = 5
    RULE_vector = 6
    RULE_string = 7
    RULE_array = 8
    RULE_vectorWithNewlines = 9
    RULE_boolean = 10

    ruleNames =  [ "world", "defn", "node", "nodeBody", "attribute", "value", 
                   "vector", "string", "array", "vectorWithNewlines", "boolean" ]

    EOF = Token.EOF
    T__0=1
    T__1=2
    T__2=3
    T__3=4
    T__4=5
    T__5=6
    T__6=7
    T__7=8
    T__8=9
    Comment=10
    Whitespace=11
    Identifier=12
    Number=13
    String=14
    Newline=15

    def __init__(self, input:TokenStream, output:TextIO = sys.stdout):
        super().__init__(input, output)
        self.checkVersion("4.11.1")
        self._interp = ParserATNSimulator(self, self.atn, self.decisionsToDFA, self.sharedContextCache)
        self._predicates = None




    class WorldContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def node(self, i:int=None):
            if i is None:
                return self.getTypedRuleContexts(WBTParser.NodeContext)
            else:
                return self.getTypedRuleContext(WBTParser.NodeContext,i)


        def defn(self, i:int=None):
            if i is None:
                return self.getTypedRuleContexts(WBTParser.DefnContext)
            else:
                return self.getTypedRuleContext(WBTParser.DefnContext,i)


        def getRuleIndex(self):
            return WBTParser.RULE_world

        def accept(self, visitor:ParseTreeVisitor):
            if hasattr( visitor, "visitWorld" ):
                return visitor.visitWorld(self)
            else:
                return visitor.visitChildren(self)




    def world(self):

        localctx = WBTParser.WorldContext(self, self._ctx, self.state)
        self.enterRule(localctx, 0, self.RULE_world)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 26
            self._errHandler.sync(self)
            _la = self._input.LA(1)
            while _la==1 or _la==12:
                self.state = 24
                self._errHandler.sync(self)
                token = self._input.LA(1)
                if token in [12]:
                    self.state = 22
                    self.node()
                    pass
                elif token in [1]:
                    self.state = 23
                    self.defn()
                    pass
                else:
                    raise NoViableAltException(self)

                self.state = 28
                self._errHandler.sync(self)
                _la = self._input.LA(1)

        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class DefnContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def Identifier(self):
            return self.getToken(WBTParser.Identifier, 0)

        def node(self):
            return self.getTypedRuleContext(WBTParser.NodeContext,0)


        def getRuleIndex(self):
            return WBTParser.RULE_defn

        def accept(self, visitor:ParseTreeVisitor):
            if hasattr( visitor, "visitDefn" ):
                return visitor.visitDefn(self)
            else:
                return visitor.visitChildren(self)




    def defn(self):

        localctx = WBTParser.DefnContext(self, self._ctx, self.state)
        self.enterRule(localctx, 2, self.RULE_defn)
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 29
            self.match(WBTParser.T__0)
            self.state = 30
            self.match(WBTParser.Identifier)
            self.state = 31
            self.node()
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class NodeContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def Identifier(self):
            return self.getToken(WBTParser.Identifier, 0)

        def Newline(self, i:int=None):
            if i is None:
                return self.getTokens(WBTParser.Newline)
            else:
                return self.getToken(WBTParser.Newline, i)

        def nodeBody(self):
            return self.getTypedRuleContext(WBTParser.NodeBodyContext,0)


        def getRuleIndex(self):
            return WBTParser.RULE_node

        def accept(self, visitor:ParseTreeVisitor):
            if hasattr( visitor, "visitNode" ):
                return visitor.visitNode(self)
            else:
                return visitor.visitChildren(self)




    def node(self):

        localctx = WBTParser.NodeContext(self, self._ctx, self.state)
        self.enterRule(localctx, 4, self.RULE_node)
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 33
            self.match(WBTParser.Identifier)
            self.state = 34
            self.match(WBTParser.T__1)
            self.state = 35
            self.match(WBTParser.Newline)
            self.state = 36
            self.nodeBody()
            self.state = 37
            self.match(WBTParser.T__2)
            self.state = 39
            self._errHandler.sync(self)
            la_ = self._interp.adaptivePredict(self._input,2,self._ctx)
            if la_ == 1:
                self.state = 38
                self.match(WBTParser.Newline)


        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class NodeBodyContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def attribute(self, i:int=None):
            if i is None:
                return self.getTypedRuleContexts(WBTParser.AttributeContext)
            else:
                return self.getTypedRuleContext(WBTParser.AttributeContext,i)


        def Newline(self, i:int=None):
            if i is None:
                return self.getTokens(WBTParser.Newline)
            else:
                return self.getToken(WBTParser.Newline, i)

        def getRuleIndex(self):
            return WBTParser.RULE_nodeBody

        def accept(self, visitor:ParseTreeVisitor):
            if hasattr( visitor, "visitNodeBody" ):
                return visitor.visitNodeBody(self)
            else:
                return visitor.visitChildren(self)




    def nodeBody(self):

        localctx = WBTParser.NodeBodyContext(self, self._ctx, self.state)
        self.enterRule(localctx, 6, self.RULE_nodeBody)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 46
            self._errHandler.sync(self)
            _la = self._input.LA(1)
            while _la==4 or _la==12:
                self.state = 41
                self.attribute()
                self.state = 42
                self.match(WBTParser.Newline)
                self.state = 48
                self._errHandler.sync(self)
                _la = self._input.LA(1)

        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class AttributeContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def Identifier(self):
            return self.getToken(WBTParser.Identifier, 0)

        def value(self):
            return self.getTypedRuleContext(WBTParser.ValueContext,0)


        def getRuleIndex(self):
            return WBTParser.RULE_attribute

        def accept(self, visitor:ParseTreeVisitor):
            if hasattr( visitor, "visitAttribute" ):
                return visitor.visitAttribute(self)
            else:
                return visitor.visitChildren(self)




    def attribute(self):

        localctx = WBTParser.AttributeContext(self, self._ctx, self.state)
        self.enterRule(localctx, 8, self.RULE_attribute)
        try:
            self.state = 54
            self._errHandler.sync(self)
            token = self._input.LA(1)
            if token in [4]:
                self.enterOuterAlt(localctx, 1)
                self.state = 49
                self.match(WBTParser.T__3)
                self.state = 50
                self.match(WBTParser.Identifier)
                self.state = 51
                self.value()
                pass
            elif token in [12]:
                self.enterOuterAlt(localctx, 2)
                self.state = 52
                self.match(WBTParser.Identifier)
                self.state = 53
                self.value()
                pass
            else:
                raise NoViableAltException(self)

        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class ValueContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def vector(self):
            return self.getTypedRuleContext(WBTParser.VectorContext,0)


        def string(self):
            return self.getTypedRuleContext(WBTParser.StringContext,0)


        def array(self):
            return self.getTypedRuleContext(WBTParser.ArrayContext,0)


        def node(self):
            return self.getTypedRuleContext(WBTParser.NodeContext,0)


        def boolean(self):
            return self.getTypedRuleContext(WBTParser.BooleanContext,0)


        def getRuleIndex(self):
            return WBTParser.RULE_value

        def accept(self, visitor:ParseTreeVisitor):
            if hasattr( visitor, "visitValue" ):
                return visitor.visitValue(self)
            else:
                return visitor.visitChildren(self)




    def value(self):

        localctx = WBTParser.ValueContext(self, self._ctx, self.state)
        self.enterRule(localctx, 10, self.RULE_value)
        try:
            self.state = 61
            self._errHandler.sync(self)
            token = self._input.LA(1)
            if token in [13]:
                self.enterOuterAlt(localctx, 1)
                self.state = 56
                self.vector()
                pass
            elif token in [14]:
                self.enterOuterAlt(localctx, 2)
                self.state = 57
                self.string()
                pass
            elif token in [5]:
                self.enterOuterAlt(localctx, 3)
                self.state = 58
                self.array()
                pass
            elif token in [12]:
                self.enterOuterAlt(localctx, 4)
                self.state = 59
                self.node()
                pass
            elif token in [8, 9]:
                self.enterOuterAlt(localctx, 5)
                self.state = 60
                self.boolean()
                pass
            else:
                raise NoViableAltException(self)

        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class VectorContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def Number(self, i:int=None):
            if i is None:
                return self.getTokens(WBTParser.Number)
            else:
                return self.getToken(WBTParser.Number, i)

        def getRuleIndex(self):
            return WBTParser.RULE_vector

        def accept(self, visitor:ParseTreeVisitor):
            if hasattr( visitor, "visitVector" ):
                return visitor.visitVector(self)
            else:
                return visitor.visitChildren(self)




    def vector(self):

        localctx = WBTParser.VectorContext(self, self._ctx, self.state)
        self.enterRule(localctx, 12, self.RULE_vector)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 64 
            self._errHandler.sync(self)
            _la = self._input.LA(1)
            while True:
                self.state = 63
                self.match(WBTParser.Number)
                self.state = 66 
                self._errHandler.sync(self)
                _la = self._input.LA(1)
                if not (_la==13):
                    break

        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class StringContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def String(self):
            return self.getToken(WBTParser.String, 0)

        def getRuleIndex(self):
            return WBTParser.RULE_string

        def accept(self, visitor:ParseTreeVisitor):
            if hasattr( visitor, "visitString" ):
                return visitor.visitString(self)
            else:
                return visitor.visitChildren(self)




    def string(self):

        localctx = WBTParser.StringContext(self, self._ctx, self.state)
        self.enterRule(localctx, 14, self.RULE_string)
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 68
            self.match(WBTParser.String)
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class ArrayContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def Newline(self, i:int=None):
            if i is None:
                return self.getTokens(WBTParser.Newline)
            else:
                return self.getToken(WBTParser.Newline, i)

        def value(self, i:int=None):
            if i is None:
                return self.getTypedRuleContexts(WBTParser.ValueContext)
            else:
                return self.getTypedRuleContext(WBTParser.ValueContext,i)


        def vectorWithNewlines(self, i:int=None):
            if i is None:
                return self.getTypedRuleContexts(WBTParser.VectorWithNewlinesContext)
            else:
                return self.getTypedRuleContext(WBTParser.VectorWithNewlinesContext,i)


        def getRuleIndex(self):
            return WBTParser.RULE_array

        def accept(self, visitor:ParseTreeVisitor):
            if hasattr( visitor, "visitArray" ):
                return visitor.visitArray(self)
            else:
                return visitor.visitChildren(self)




    def array(self):

        localctx = WBTParser.ArrayContext(self, self._ctx, self.state)
        self.enterRule(localctx, 16, self.RULE_array)
        self._la = 0 # Token type
        try:
            self.state = 95
            self._errHandler.sync(self)
            la_ = self._interp.adaptivePredict(self._input,11,self._ctx)
            if la_ == 1:
                self.enterOuterAlt(localctx, 1)
                self.state = 70
                self.match(WBTParser.T__4)
                self.state = 75
                self._errHandler.sync(self)
                _alt = self._interp.adaptivePredict(self._input,7,self._ctx)
                while _alt!=2 and _alt!=ATN.INVALID_ALT_NUMBER:
                    if _alt==1:
                        self.state = 71
                        self.match(WBTParser.Newline)
                        self.state = 72
                        self.value() 
                    self.state = 77
                    self._errHandler.sync(self)
                    _alt = self._interp.adaptivePredict(self._input,7,self._ctx)

                self.state = 79
                self._errHandler.sync(self)
                _la = self._input.LA(1)
                if _la==15:
                    self.state = 78
                    self.match(WBTParser.Newline)


                self.state = 81
                self.match(WBTParser.T__5)
                pass

            elif la_ == 2:
                self.enterOuterAlt(localctx, 2)
                self.state = 82
                self.match(WBTParser.T__4)
                self.state = 83
                self.vectorWithNewlines()
                self.state = 86 
                self._errHandler.sync(self)
                _alt = 1
                while _alt!=2 and _alt!=ATN.INVALID_ALT_NUMBER:
                    if _alt == 1:
                        self.state = 84
                        self.match(WBTParser.T__6)
                        self.state = 85
                        self.vectorWithNewlines()

                    else:
                        raise NoViableAltException(self)
                    self.state = 88 
                    self._errHandler.sync(self)
                    _alt = self._interp.adaptivePredict(self._input,9,self._ctx)

                self.state = 91
                self._errHandler.sync(self)
                _la = self._input.LA(1)
                if _la==7:
                    self.state = 90
                    self.match(WBTParser.T__6)


                self.state = 93
                self.match(WBTParser.T__5)
                pass


        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class VectorWithNewlinesContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def Number(self, i:int=None):
            if i is None:
                return self.getTokens(WBTParser.Number)
            else:
                return self.getToken(WBTParser.Number, i)

        def Newline(self, i:int=None):
            if i is None:
                return self.getTokens(WBTParser.Newline)
            else:
                return self.getToken(WBTParser.Newline, i)

        def getRuleIndex(self):
            return WBTParser.RULE_vectorWithNewlines

        def accept(self, visitor:ParseTreeVisitor):
            if hasattr( visitor, "visitVectorWithNewlines" ):
                return visitor.visitVectorWithNewlines(self)
            else:
                return visitor.visitChildren(self)




    def vectorWithNewlines(self):

        localctx = WBTParser.VectorWithNewlinesContext(self, self._ctx, self.state)
        self.enterRule(localctx, 18, self.RULE_vectorWithNewlines)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 104 
            self._errHandler.sync(self)
            _la = self._input.LA(1)
            while True:
                self.state = 98
                self._errHandler.sync(self)
                _la = self._input.LA(1)
                if _la==15:
                    self.state = 97
                    self.match(WBTParser.Newline)


                self.state = 100
                self.match(WBTParser.Number)
                self.state = 102
                self._errHandler.sync(self)
                la_ = self._interp.adaptivePredict(self._input,13,self._ctx)
                if la_ == 1:
                    self.state = 101
                    self.match(WBTParser.Newline)


                self.state = 106 
                self._errHandler.sync(self)
                _la = self._input.LA(1)
                if not (_la==13 or _la==15):
                    break

        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class BooleanContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser


        def getRuleIndex(self):
            return WBTParser.RULE_boolean

        def accept(self, visitor:ParseTreeVisitor):
            if hasattr( visitor, "visitBoolean" ):
                return visitor.visitBoolean(self)
            else:
                return visitor.visitChildren(self)




    def boolean(self):

        localctx = WBTParser.BooleanContext(self, self._ctx, self.state)
        self.enterRule(localctx, 20, self.RULE_boolean)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 108
            _la = self._input.LA(1)
            if not(_la==8 or _la==9):
                self._errHandler.recoverInline(self)
            else:
                self._errHandler.reportMatch(self)
                self.consume()
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx






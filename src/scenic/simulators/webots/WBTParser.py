# Generated from WBT.g4 by ANTLR 4.7.1
# encoding: utf-8
from antlr4 import *
from io import StringIO
from typing.io import TextIO
import sys

def serializedATN():
    with StringIO() as buf:
        buf.write("\3\u608b\ua72a\u8133\ub9ed\u417c\u3be7\u7786\u5964\3\21")
        buf.write("n\4\2\t\2\4\3\t\3\4\4\t\4\4\5\t\5\4\6\t\6\4\7\t\7\4\b")
        buf.write("\t\b\4\t\t\t\4\n\t\n\4\13\t\13\4\f\t\f\3\2\3\2\7\2\33")
        buf.write("\n\2\f\2\16\2\36\13\2\3\3\3\3\3\3\3\3\3\4\3\4\3\4\3\4")
        buf.write("\3\4\3\4\5\4*\n\4\3\5\3\5\3\5\7\5/\n\5\f\5\16\5\62\13")
        buf.write("\5\3\6\3\6\3\6\3\6\3\6\5\69\n\6\3\7\3\7\3\7\3\7\3\7\5")
        buf.write("\7@\n\7\3\b\6\bC\n\b\r\b\16\bD\3\t\3\t\3\n\3\n\3\n\7\n")
        buf.write("L\n\n\f\n\16\nO\13\n\3\n\5\nR\n\n\3\n\3\n\3\n\3\n\3\n")
        buf.write("\6\nY\n\n\r\n\16\nZ\3\n\3\n\5\n_\n\n\3\13\5\13b\n\13\3")
        buf.write("\13\3\13\5\13f\n\13\6\13h\n\13\r\13\16\13i\3\f\3\f\3\f")
        buf.write("\2\2\r\2\4\6\b\n\f\16\20\22\24\26\2\3\3\2\n\13\2s\2\34")
        buf.write("\3\2\2\2\4\37\3\2\2\2\6#\3\2\2\2\b\60\3\2\2\2\n8\3\2\2")
        buf.write("\2\f?\3\2\2\2\16B\3\2\2\2\20F\3\2\2\2\22^\3\2\2\2\24g")
        buf.write("\3\2\2\2\26k\3\2\2\2\30\33\5\6\4\2\31\33\5\4\3\2\32\30")
        buf.write("\3\2\2\2\32\31\3\2\2\2\33\36\3\2\2\2\34\32\3\2\2\2\34")
        buf.write("\35\3\2\2\2\35\3\3\2\2\2\36\34\3\2\2\2\37 \7\3\2\2 !\7")
        buf.write("\16\2\2!\"\5\6\4\2\"\5\3\2\2\2#$\7\16\2\2$%\7\4\2\2%&")
        buf.write("\7\21\2\2&\'\5\b\5\2\')\7\5\2\2(*\7\21\2\2)(\3\2\2\2)")
        buf.write("*\3\2\2\2*\7\3\2\2\2+,\5\n\6\2,-\7\21\2\2-/\3\2\2\2.+")
        buf.write("\3\2\2\2/\62\3\2\2\2\60.\3\2\2\2\60\61\3\2\2\2\61\t\3")
        buf.write("\2\2\2\62\60\3\2\2\2\63\64\7\6\2\2\64\65\7\16\2\2\659")
        buf.write("\5\f\7\2\66\67\7\16\2\2\679\5\f\7\28\63\3\2\2\28\66\3")
        buf.write("\2\2\29\13\3\2\2\2:@\5\16\b\2;@\5\20\t\2<@\5\22\n\2=@")
        buf.write("\5\6\4\2>@\5\26\f\2?:\3\2\2\2?;\3\2\2\2?<\3\2\2\2?=\3")
        buf.write("\2\2\2?>\3\2\2\2@\r\3\2\2\2AC\7\17\2\2BA\3\2\2\2CD\3\2")
        buf.write("\2\2DB\3\2\2\2DE\3\2\2\2E\17\3\2\2\2FG\7\20\2\2G\21\3")
        buf.write("\2\2\2HM\7\7\2\2IJ\7\21\2\2JL\5\f\7\2KI\3\2\2\2LO\3\2")
        buf.write("\2\2MK\3\2\2\2MN\3\2\2\2NQ\3\2\2\2OM\3\2\2\2PR\7\21\2")
        buf.write("\2QP\3\2\2\2QR\3\2\2\2RS\3\2\2\2S_\7\b\2\2TU\7\7\2\2U")
        buf.write("X\5\24\13\2VW\7\t\2\2WY\5\24\13\2XV\3\2\2\2YZ\3\2\2\2")
        buf.write("ZX\3\2\2\2Z[\3\2\2\2[\\\3\2\2\2\\]\7\b\2\2]_\3\2\2\2^")
        buf.write("H\3\2\2\2^T\3\2\2\2_\23\3\2\2\2`b\7\21\2\2a`\3\2\2\2a")
        buf.write("b\3\2\2\2bc\3\2\2\2ce\7\17\2\2df\7\21\2\2ed\3\2\2\2ef")
        buf.write("\3\2\2\2fh\3\2\2\2ga\3\2\2\2hi\3\2\2\2ig\3\2\2\2ij\3\2")
        buf.write("\2\2j\25\3\2\2\2kl\t\2\2\2l\27\3\2\2\2\20\32\34)\608?")
        buf.write("DMQZ^aei")
        return buf.getvalue()


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
        self.checkVersion("4.7.1")
        self._interp = ParserATNSimulator(self, self.atn, self.decisionsToDFA, self.sharedContextCache)
        self._predicates = None



    class WorldContext(ParserRuleContext):

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

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterWorld" ):
                listener.enterWorld(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitWorld" ):
                listener.exitWorld(self)

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
            while _la==WBTParser.T__0 or _la==WBTParser.Identifier:
                self.state = 24
                self._errHandler.sync(self)
                token = self._input.LA(1)
                if token in [WBTParser.Identifier]:
                    self.state = 22
                    self.node()
                    pass
                elif token in [WBTParser.T__0]:
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

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def Identifier(self):
            return self.getToken(WBTParser.Identifier, 0)

        def node(self):
            return self.getTypedRuleContext(WBTParser.NodeContext,0)


        def getRuleIndex(self):
            return WBTParser.RULE_defn

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterDefn" ):
                listener.enterDefn(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitDefn" ):
                listener.exitDefn(self)

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

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterNode" ):
                listener.enterNode(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitNode" ):
                listener.exitNode(self)

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

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterNodeBody" ):
                listener.enterNodeBody(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitNodeBody" ):
                listener.exitNodeBody(self)

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
            while _la==WBTParser.T__3 or _la==WBTParser.Identifier:
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

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def Identifier(self):
            return self.getToken(WBTParser.Identifier, 0)

        def value(self):
            return self.getTypedRuleContext(WBTParser.ValueContext,0)


        def getRuleIndex(self):
            return WBTParser.RULE_attribute

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterAttribute" ):
                listener.enterAttribute(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitAttribute" ):
                listener.exitAttribute(self)

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
            if token in [WBTParser.T__3]:
                self.enterOuterAlt(localctx, 1)
                self.state = 49
                self.match(WBTParser.T__3)
                self.state = 50
                self.match(WBTParser.Identifier)
                self.state = 51
                self.value()
                pass
            elif token in [WBTParser.Identifier]:
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

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterValue" ):
                listener.enterValue(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitValue" ):
                listener.exitValue(self)

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
            if token in [WBTParser.Number]:
                self.enterOuterAlt(localctx, 1)
                self.state = 56
                self.vector()
                pass
            elif token in [WBTParser.String]:
                self.enterOuterAlt(localctx, 2)
                self.state = 57
                self.string()
                pass
            elif token in [WBTParser.T__4]:
                self.enterOuterAlt(localctx, 3)
                self.state = 58
                self.array()
                pass
            elif token in [WBTParser.Identifier]:
                self.enterOuterAlt(localctx, 4)
                self.state = 59
                self.node()
                pass
            elif token in [WBTParser.T__7, WBTParser.T__8]:
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

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterVector" ):
                listener.enterVector(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitVector" ):
                listener.exitVector(self)

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
                if not (_la==WBTParser.Number):
                    break

        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx

    class StringContext(ParserRuleContext):

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def String(self):
            return self.getToken(WBTParser.String, 0)

        def getRuleIndex(self):
            return WBTParser.RULE_string

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterString" ):
                listener.enterString(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitString" ):
                listener.exitString(self)

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

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterArray" ):
                listener.enterArray(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitArray" ):
                listener.exitArray(self)

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
            self.state = 92
            self._errHandler.sync(self)
            la_ = self._interp.adaptivePredict(self._input,10,self._ctx)
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
                if _la==WBTParser.Newline:
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
                _la = self._input.LA(1)
                while True:
                    self.state = 84
                    self.match(WBTParser.T__6)
                    self.state = 85
                    self.vectorWithNewlines()
                    self.state = 88 
                    self._errHandler.sync(self)
                    _la = self._input.LA(1)
                    if not (_la==WBTParser.T__6):
                        break

                self.state = 90
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

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterVectorWithNewlines" ):
                listener.enterVectorWithNewlines(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitVectorWithNewlines" ):
                listener.exitVectorWithNewlines(self)

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
            self.state = 101 
            self._errHandler.sync(self)
            _la = self._input.LA(1)
            while True:
                self.state = 95
                self._errHandler.sync(self)
                _la = self._input.LA(1)
                if _la==WBTParser.Newline:
                    self.state = 94
                    self.match(WBTParser.Newline)


                self.state = 97
                self.match(WBTParser.Number)
                self.state = 99
                self._errHandler.sync(self)
                la_ = self._interp.adaptivePredict(self._input,12,self._ctx)
                if la_ == 1:
                    self.state = 98
                    self.match(WBTParser.Newline)


                self.state = 103 
                self._errHandler.sync(self)
                _la = self._input.LA(1)
                if not (_la==WBTParser.Number or _la==WBTParser.Newline):
                    break

        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx

    class BooleanContext(ParserRuleContext):

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser


        def getRuleIndex(self):
            return WBTParser.RULE_boolean

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterBoolean" ):
                listener.enterBoolean(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitBoolean" ):
                listener.exitBoolean(self)

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
            self.state = 105
            _la = self._input.LA(1)
            if not(_la==WBTParser.T__7 or _la==WBTParser.T__8):
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






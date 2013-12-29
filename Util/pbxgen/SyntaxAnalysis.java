class SyntaxAnalysis extends LexicalAnalysis {
    private void unexpectedSymbol() throws Exception {
        throw new Exception("Unexpected symbol " + symbol.toString());
    }
    
    private void expectSymbol(Symbol expected) throws Exception {
        if (expected != symbol) {
            unexpectedSymbol();
        }
        nextSymbol();
    }
    
    private Literal expectLiteral() throws Exception {
        if (symbol != Symbol.LITERAL) {
            unexpectedSymbol();
        }
        String s = string;
        nextSymbol();
        return new Literal(s, acceptComment());
    }
    
    private String acceptComment() throws Exception {
        if (symbol == Symbol.COMMENT) {
            String s = string;
            nextSymbol();
            return s;
        } else {
            return null;
        }
    }

    private Record record() throws Exception {
        newLine = false;
        expectSymbol(Symbol.LBRACE);
        Record r = new Record();
        while (symbol == Symbol.LITERAL || symbol == Symbol.COMMENT) {
            while (symbol == Symbol.COMMENT) {
                String c = acceptComment();
                if (c != null) {
                    r.values.add(new Comment(c));
                }
            }
            if (symbol == Symbol.LITERAL) {
                r.values.add(attribute());
                expectSymbol(Symbol.SEMICOLON);
            }
        }
        r.multiLine = newLine;
        expectSymbol(Symbol.RBRACE);
        return r;
    }

    private Attribute attribute() throws Exception {
        Literal l = expectLiteral();
        expectSymbol(Symbol.EQUALS);
        if (symbol == Symbol.LITERAL) {
            return new Attribute(l, expectLiteral());
        } else if (symbol == Symbol.LBRACE) {
            return new Attribute(l, record());
        } else if (symbol == Symbol.LPAREN) {
            return new Attribute(l, list());
        } else {
            throw new Exception("Unexpected symbol " + symbol.toString());
        }
    }

    private List list() throws Exception {
        expectSymbol(Symbol.LPAREN);
        List l = new List();
        while (symbol == Symbol.LITERAL) {
            l.literals.add(expectLiteral());
            expectSymbol(Symbol.COMMA);
        }
        expectSymbol(Symbol.RPAREN);
        return l;
    }

    SyntaxAnalysis(String fileName, boolean printSymbols, boolean printSource) throws Exception {
        super(fileName, printSymbols, printSource);
    }

    /**
     * Die Methode parsiert den Quelltext und liefert die Wurzel des 
     * Syntaxbaums zurück.
     * @throws CompileException Der Quelltext entspricht nicht der Syntax.
     * @throws IOException Ein Lesefehler ist aufgetreten.
     */
    Record parse() throws Exception {
        nextSymbol();
        Record r = record();
        expectSymbol(Symbol.EOF);
        return r;
    }
}

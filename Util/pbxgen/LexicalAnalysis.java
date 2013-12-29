import java.io.FileInputStream;
import java.io.InputStreamReader;
import java.io.IOException;


class LexicalAnalysis {
    private InputStreamReader reader;
    private boolean printSymbols;
    private boolean printSource;
    private int c;
    
    enum Symbol {
        LITERAL, COMMENT, EQUALS,
        COMMA, SEMICOLON, 
        LPAREN, RPAREN,
        LBRACE, RBRACE,
        EOF
    };
    
    Symbol symbol;
    String string;
    boolean newLine;

    private void nextChar() throws IOException {
        if (c == '\n') {
            newLine = true;
        }
        c = reader.read();
        if (printSource && c != -1) {
            System.out.print((char) c);
        }
    }
    
    LexicalAnalysis(String fileName, boolean printSymbols, boolean printSource) 
            throws IOException {
        FileInputStream stream = new FileInputStream(fileName);
        reader = new InputStreamReader(stream);
        this.printSymbols = printSymbols;
        this.printSource = printSource;

        nextChar();
    }
    
    void close() throws IOException {
        reader.close();
    }
    
    void nextSymbol() throws Exception {
        for (;;) {
            // Leerraum ignorieren
            while (c != -1 && Character.isWhitespace((char) c)) {
                nextChar();
            }
        
            string = "";
            string = "";
            switch (c) {
            case -1:
                symbol = Symbol.EOF;
                break;
            case '=':
                symbol = Symbol.EQUALS;
                nextChar();
                break;
            case ',':
                symbol = Symbol.COMMA;
                nextChar();
                break;
            case ';':
                symbol = Symbol.SEMICOLON;
                nextChar();
                break;
            case '(':
                symbol = Symbol.LPAREN;
                nextChar();
                break;
            case ')':
                symbol = Symbol.RPAREN;
                nextChar();
                break;
            case '{':
                symbol = Symbol.LBRACE;
                nextChar();
                break;
            case '}':
                symbol = Symbol.RBRACE;
                nextChar();
                break;
            case '"':
                string = "" + (char) c;
                nextChar();
                while (c != -1 && c != '"') {
                    string += (char) c;
                    if (c == '\\') {
                        nextChar();
                        string += (char) c;
                    }
                    nextChar();
                }
                if (c == -1) {
                    throw new Exception("Unexpected EOF in string");
                }
                string += (char) c;
                nextChar();
                symbol = Symbol.LITERAL;
                break;
                
            case '/':
                nextChar();
                if (c == '*') {
                    nextChar();
                    string = "/*";
                    int prevChar = -1;
                    while (c != -1 && (c != '/' || prevChar != '*')) {
                        prevChar = c;
                        string += (char) c;
                        nextChar();
                    }
                    if (c == -1) {
                        throw new Exception("Unexpected EOF in comment");
                    }
                    string += (char) c;
                    nextChar();
                    symbol = Symbol.COMMENT;
                    break;
                } else if (c == '/') {
                    nextChar();
                    while (c != -1 && c != '\n') {
                        nextChar();
                    }
                    if (c != -1) {
                        nextChar();
                    }
                    continue;
                }
                string = "/";
                // no break;
    
            default:
                while (c != -1 && !Character.isWhitespace((char) c) && c != ',' && c != ';') {
                    string += (char) c;
                    nextChar();
                }
                symbol = Symbol.LITERAL;
            }
            break;
        }
        if (printSymbols) {
            System.out.println(symbol == Symbol.LITERAL || symbol == Symbol.COMMENT ? string : symbol.toString());
        }
    }
}

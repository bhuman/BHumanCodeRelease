import java.io.OutputStream;
import java.io.PrintStream;

/**
 * Die Klasse realisiert einen Ausgabestrom, in dem Text baumartig strukturiert
 * werden kann. Da die Klasse von {@link java.io.PrintStream PrintStream} 
 * erbt, können alle Methoden verwendet werden, mit denen man auch auf die 
 * Konsole schreiben kann. Zusätzlich gibt es Methoden zum Steuern der
 * Einrückungstiefe.
 */
class TreeStream extends PrintStream {
    /** 
     * Ein Puffer für das zuletzt ausgegebene Zeichen. Falls das letzte Zeichen
     * ein '\n' war, wird vor der Ausgabe des nächsten Zeichens eingerückt.
     */
    private int lastChar = 0;
    
    /** Die aktuelle Einrücktiefe. */
    private int indention = 0;
    
    /**
     * Konstruktor.
     * @param stream Der Ausgabestrom, in den geschrieben wird.
     */
    TreeStream(OutputStream stream) {
        super(stream);
    }
    
    /**
     * Die Methode erhöht die Einrücktiefe der Ausgabe.
     */
    void indent() {
        ++indention;
    }
    
    /**
     * Die Methode verringert die Einrücktiefe der Ausgabe.
     */
    void unindent() {
        --indention;
        assert indention >= 0;
    }
    
    /**
     * Die Methode überschreibt die Ausgabemethode der Basisklasse.
     * Sie stellt sicher, dass die Einrückungen vorgenommen werden.
     * @param buf Der Puffer, der ausgegeben werden soll.
     * @param off Der Index des ersten Zeichens in dem Puffer, das 
     *         ausgegeben werden soll.
     * @param len Die Anzahl der Zeichen, die ausgegeben werden sollen.
     */
    public void write(byte[] buf, int off, int len) {
        for (int i = 0; i < len; ++i) {
            write((int) buf[off + i]);
        }
    }
    
    /**
     * Die Methode überschreibt die Ausgabemethode der Basisklasse.
     * Sie stellt sicher, dass die Einrückungen vorgenommen werden.
     * @param b Das auszugebene Zeichen.
     */
    public void write(int b) {
        if (lastChar == '\n') {
            for (int i = 0; i < indention; ++i) {
                super.write('\t');
            }
        }
        lastChar = b;
        super.write(b);
    }
}

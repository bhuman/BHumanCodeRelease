import java.util.LinkedList;

class List extends Value {
    LinkedList<Literal> literals = new LinkedList<Literal>();
    
    void print(TreeStream stream) {
        stream.println("(");
        stream.indent();
        for (Literal l : literals) {
            l.print(stream);
            stream.println(",");
        }
        stream.unindent();
        stream.print(")");
    }
}

import java.util.LinkedList;

class Record extends Value {
    LinkedList<Value> values = new LinkedList<Value>();
    boolean multiLine;
    
    Value get(String key) {
        for (Value f : values) {
            if (f instanceof Attribute && ((Attribute) f).key.literal.equals(key)) {
                return ((Attribute) f).value;
            }
        }
        return null;
    }

    void print(TreeStream stream) {
        char separator;
        if (multiLine) {
            separator = '\n';
            stream.println("{");
        } else {
            separator = ' ';
            stream.print("{");
        }
        stream.indent();
        boolean lastWasComment = true;
        for (Value v : values) {
            if (v instanceof Comment && lastWasComment) {
                new Comment("").print(stream);
            }
            v.print(stream);
            if (v instanceof Attribute) {
                stream.print(";" + separator);
                lastWasComment = false;
            } else {
                lastWasComment = true;
            }
        }
        stream.unindent();
        stream.print("}");
    }
}

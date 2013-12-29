class Attribute extends Value implements Comparable<Attribute> {
    Literal key;
    Value value;
    
    Attribute(Literal key, Value value) {
        this.key = key;
        this.value = value;
    }

    void print(TreeStream stream) {
        key.print(stream);
        stream.print(" = ");
        value.print(stream);
    }
    
    public int compareTo(Attribute other) {
        return key.literal.compareTo(other.key.literal);
    }
}

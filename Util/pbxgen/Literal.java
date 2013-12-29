class Literal extends Value implements Comparable<Literal> {
    String literal;
    String comment;
    
    Literal(String literal) {
        this.literal = literal;
    }
    
    Literal(String literal, String comment) {
        this.literal = literal;
        this.comment = comment;
    }
    
    void print(TreeStream stream) {
        stream.print(literal);
        if (comment != null) {
            stream.print(" " + comment);
        }
    }
    
    public boolean equals(Object other) {
        return other instanceof Literal &&
                literal.equals(((Literal) other).literal);
    }
    
    public int compareTo(Literal other) {
        return literal.compareTo(other.literal);
    }

}
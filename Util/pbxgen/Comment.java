class Comment extends Value {
    String comment;
    
    Comment(String comment) {
        this.comment = comment;
    }
    
    void print(TreeStream stream) {
        stream.unindent();
        stream.unindent();
        stream.println(comment);
        stream.indent();
        stream.indent();
   }
}

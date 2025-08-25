
class Tuple<A, B> {
    private A idx;
    private B value;

    public Tuple(A idx, B value) {
        this.idx = idx;
        this.value = value;
    }

    public A getIdx() {
        return idx;
    }

    public B getValue() {
        return value;
    }
}


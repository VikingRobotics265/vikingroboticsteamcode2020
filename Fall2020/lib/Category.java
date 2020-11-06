package org.firstinspires.ftc.teamcode.vikingroboticsteamcode2020.Fall2020.lib;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.teamcode.BuildConfig;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import static java.lang.Math.*;

interface Proxy extends Comparable<Proxy> {
}
class PureProxy implements Proxy{
    public Class<?> content;
    PureProxy(Class<?> content) {
        this.content = content;
    }
    @Override
    public boolean equals(Object proxy) {
        return (proxy instanceof PureProxy) && (this.content == ((PureProxy) proxy).content);
    }
    // throws exception if argument proxy is not an PureProxy
    // return 1 ~> this super arg
    // return 0 ~> this == arg
    // return -1 ~> this extends arg
    @Override public int compareTo(Proxy proxy) throws AssertionError {
        if (BuildConfig.DEBUG && !(proxy instanceof PureProxy))
            throw new AssertionError("should be an instance of PureProxy");
        return this.equals(proxy)?0:
                this.content.isAssignableFrom((((PureProxy)proxy).content))?1:-1;
    }
}
class HomProxy implements Proxy {
    public Proxy in, out;
    HomProxy(Proxy proxyIn, Proxy proxyOut) {
        this.in = proxyIn;
        this.out = proxyOut;
    }
    @Override
    public boolean equals(Object proxy) {
        return (proxy instanceof HomProxy)
                && (this.in == ((HomProxy)proxy).in)
                && (this.out == ((HomProxy)proxy).out);
    }
    @Override
    public int compareTo(Proxy proxy) {
        if (BuildConfig.DEBUG && !(proxy instanceof HomProxy))
            throw new AssertionError("should be an instance of HomProxy");
        if (BuildConfig.DEBUG && !((
                this.in.compareTo(((HomProxy) proxy).in) *
                this.out.compareTo(((HomProxy) proxy).out) < 0))) {
            throw new AssertionError("one component should be covariant and another should be contravariant");
        }
        return this.equals(proxy)?0:
                (this.out.compareTo(((HomProxy) proxy).out) > 0)?1:-1;
    }
}
public class Category {
    public static class Unit {
        final public static Unit unit = new Unit();
        private Unit() {}
    }
    @SuppressWarnings("unchecked")
    public static <A> Class<A> assume(Class<?> aClass) {
        return (Class<A>)aClass;
    }
    public static Proxy proxy(Class<?> content) {
        return new PureProxy(content);
    }
    public static Proxy proxy(Class<?> contentIn, Class<?> contentOut) {
        return new HomProxy(new PureProxy(contentIn), new PureProxy(contentOut));
    }
    public static Proxy proxy(Proxy proxyIn, Proxy proxyOut) {
        return new HomProxy(proxyIn, proxyOut);
    }
    // morphism bifunctor
    public static abstract class Hom<A,B> implements Function<A, B> {
        public Proxy proxy;
        public Hom(Proxy proxyIn, Proxy proxyOut) {
            this.proxy = proxy(proxyIn, proxyOut);
        }
        abstract B of(A x);
        @Deprecated @Override final public B apply(A arg) {
            return of(arg);
        }
        public List<B> fmap(final Iterable<A> xs) {
            return new ArrayList<B>() {{
                for(A x : xs) {
                    add(of(x));
                }
            }};
        }
        public <T> Hom<T,B> after(final Hom<T,A> g) {
            return new Hom<T,B>(proxy((g.proxy).in, Hom.this.proxy.out)) {@Override B of(T x){
                return Hom.this.of(g.of(x));
            }};
        }
        public <T> Hom<A,T> then(final Hom<B,T> f) {
            return f.after(this);
        }
    }
    public static abstract class Eff extends Hom<Unit,Unit> implements Runnable {
        public Eff(Class<Unit> typeIn, Class<Unit> typeOut) {
            super(typeIn, typeOut);
        }
        public Eff() {
            super(Unit.class, Unit.class);
        }
        @Override
        Unit of(Unit unit) {
            run();
            return Unit.unit;
        }
    }
    public static abstract class In<A> extends Hom<A,Unit> implements Consumer<A> {
        public In(Class<A> typeIn, Class<Unit> typeOut) {
            super(typeIn, typeOut);
        }
        public In(Class<A> typeIn) {
            super(typeIn, Unit.class);
        }
        abstract void run(A x);
        @Override
        Unit of(A x) {
            run(x);
            return Unit.unit;
        }
        @Override
        public void accept(A value) {
            run(value);
        }
        public void forEach(final Iterable<A> args) {
            for(A arg : args) {
                accept(arg);
            }
        }
    }
    public static abstract class In2<A,B> extends Hom2<A,B,Unit> {
        public In2(Class<A> typeIn, Class<Hom<B, Unit>> typeOut) {
            super(typeIn, typeOut);
        }
        abstract void run(A x, B y);
        @Override
        Unit of(A x, B y) {
            return Unit.unit;
        }
    }
    public static abstract class Out<A> extends Hom<Unit,A> implements Supplier<A>, Func<A> {
        public Out(Class<Unit> typeIn, Class<A> typeOut) {
            super(typeIn, typeOut);
        }
        public Out(Class<A> typeOut) {
            super(Unit.class, typeOut);
        }
        abstract A of();
        @Deprecated @Override A of(Unit x) {return of();}
        @Deprecated @Override public A get() {return of();}
        @Deprecated @Override public A value() {return of();}
    }
    public static abstract class Hom2<A,B,C> extends Hom<A,Hom<B,C>> {
        public Hom2(Class<A> typeIn, Class<Hom<B,C>> typeOut) {
            super(typeIn, typeOut);
        }
        public Hom2(Class<A> typeIn1, Class<B> typeIn2, Class<C> typeOut) {
            super(typeIn1, Category.<Hom<B,C>>assume(Hom.class));
        }
        abstract C of(A x, B y);
        public Hom<B,C> of(final A x) {
            return new Hom<B,C>() {@Override C of(final B y) {
                return Hom2.this.of(x, y);
            }};
        }
        public <T> Hom<A,C> ap(final Hom<A,B> g) {
            return new Hom<A,C>() {@Override C of(A x) {
                return Hom2.this.of(x, g.of(x));
            }};
        }
    }
    public static abstract class Hom3<A,B,C,D> extends Hom2<A,B,Hom<C,D>> {
        abstract D of(A x, B y, C z);
        public Hom<C,D> of(final A x, final B y) {
            return new Hom<C,D>() {@Override D of(final C z) {
                return Hom3.this.of(x, y, z);
            }};
        }
    }
    public static <A,B> Hom<A,B> proxyHom(Class<A> typeIn, Class<B> typeOut) {
        return new Hom<A, B>() {
            @Deprecated @Override
            B of(A x) {
                return null;
            }
        };
    }

    public static abstract class Endo<A> extends Hom<A,A> {}
    public static abstract class Magma<A> extends Hom2<A,A,A> {}
    public static abstract class Semi<A> extends Magma<A> {/* + association rule */}
    public static abstract class Monoid<A> extends Semi<A> {
        A id;
    }

    public static <A> Endo<A> id(Class<A> typeA) {
        return new Endo<A>() {@Override A of(A x){
            return x;
        }};
    }
    public static <A,B> Hom<B,A> diag(final A x, Class<B> typeInB) {
        return new Hom<B,A>() {
            @Override
            A of(B y) {
                return x;
            }
        };
    }
    public static <A,B> Hom2<A,B,A> diag(Class<A> typeInA, Class<B> typeOutB) {
        return new Hom2<A,B,A>() {@Override A of(A x, B y){
            return x;
        }};
    }
    public static <A,B,C> Hom2<Hom2<A,B,C>,Hom<A,B>,Hom<A,C>> ap(Class<A> typeInA, Class<B> typeInB, Class<C> typeOutC) {
        return new Hom2<Hom2<A,B,C>,Hom<A,B>,Hom<A,C>>() {
            @Override Hom<A,C> of(Hom2<A,B,C> f, Hom<A,B> g){
            return f.ap(g);
        }};
    }
    public static <A,B,C> Hom2<B,A,C> flip(final Hom2<A,B,C> f) {
        return new Hom2<B,A,C>() {
            @Override
            C of(B y, A x) {
                return f.of(x, y);
            }
        };
    }
    public static <A,B,C> Hom<Hom2<A,B,C>,Hom2<B,A,C>> flip(Class<A> typeInA, Class<B> typeInB, Class<C> typeOutC) {
        return new Hom<Hom2<A,B,C>,Hom2<B,A,C>>() {
            @Override
            Hom2<B, A, C> of(final Hom2<A, B, C> f) {
                return new Hom2<B, A, C>() {
                    @Override
                    C of(B y, A x) {
                        return f.of(x, y);
                    }
                };
            }
        };
    }
    public static <A,B> Hom2<Hom<A,B>,A,B> apply() {
        return new Hom2<Hom<A, B>, A, B>() {
            @Override
            B of(Hom<A, B> f, A x) {
                return f.of(x);
            }
        };
    }
    public static <A,B> Hom2<Hom<A,B>,List<A>,List<B>> fmap(Class<A> typeIn, Class<B> typeOut) {
        return new Hom2<Hom<A,B>,List<A>,List<B>>() {
            @Override
            ArrayList<B> of(final Hom<A, B> f, final List<A> xs) {
                return new ArrayList<B>() {{
                    for(A x : xs) add(f.of(x));
                }};
            }
        };
    }

    public static <A,B,C> Hom2<Hom<A,B>,Hom<B,C>,Hom<A,C>> then() {
        return new Hom2<Hom<A,B>,Hom<B,C>,Hom<A,C>>() {
            @Override
            Hom<A, C> of(Hom<A, B> f, Hom<B, C> g) {
                return f.then(g);
            }
        };
    }
    public static <A> Monoid<Endo<A>> combl(final Class<A> typeA) {
        return new Monoid<Endo<A>>() {
            final Endo<A> id = id(typeA);
            @Override Endo<A> of(Endo<A> x, Endo<A> y){
                return (Endo<A>) x.then(y);
            }
        };
    }
    public static <A,B,C> Hom2<Hom<B,C>,Hom<A,B>,Hom<A,C>> after() {
        return new Hom2<Hom<B,C>,Hom<A,B>,Hom<A,C>>() {
            @Override
            Hom<A, C> of(Hom<B, C> g, Hom<A, B> f) {
                return g.after(f);
            }
        };
    }
    public static <A> Monoid<Endo<A>> combr(final Class<A> typeA) {
        return new Monoid<Endo<A>>() {
            final Endo<A> id = id(typeA);
            @Override Endo<A> of(Endo<A> x, Endo<A> y){
                return (Endo<A>) x.after(y);
            }
        };
    }

    public static Monoid<Boolean> conj = new Monoid<Boolean>() {
        final Boolean id = true;
        @Override Boolean of(Boolean x, Boolean y){
            return x && y;
        }
    };
    public static Monoid<Boolean> disj = new Monoid<Boolean>() {
        final Boolean id = false;
        @Override Boolean of(Boolean x, Boolean y){
            return x || y;
        }
    };
    public static Endo<Boolean> not = new Endo<Boolean>() {
        @Override
        Boolean of(Boolean x) {
            return !x;
        }
    };

    public static Monoid<Integer> add = new Monoid<Integer>() {
        final Integer id = 0;
        @Override Integer of(Integer x, Integer y){
            return x + y;
        }
    };
    public static Monoid<Integer> mult = new Monoid<Integer>() {
        final Integer id = 1;
        @Override Integer of(Integer x, Integer y){
            return x * y;
        }
    };
    public static Magma<Integer> pow = new Magma<Integer>() {
        @Override
        Integer of(Integer x, Integer y) {
            int outp = 1;
            for(int i = 0; i < y; i++) outp *= x;
            return outp;
        }
    };
    public static Magma<Double> power = new Magma<Double>() {
        @Override
        Double of(Double x, Double y) {
            return pow(x, y);
        }
    };
    public static Endo<Integer> negative = new Endo<Integer>() {
        @Override
        Integer of(Integer x) {
            return -x;
        }
    };
    public static Endo<Double> reciprocal = new Endo<Double>() {
        @Override
        Double of(Double x) {
            return 1/x;
        }
    };
    public static Endo<Integer> sqr = new Endo<Integer>() {
        @Override
        Integer of(Integer x){
        return x * x;
    }};

    public static <A,B> A foldl(Hom2<A,B,A> f, A z, Iterable<B> xs) {
        A outp = z;
        for(B x : xs) outp = f.of(outp).of(x);
        return outp;
    }
    public static <A,B> Hom3<Hom2<A,B,A>,A,Iterable<B>,A> foldl() {
        return new Hom3<Hom2<A,B,A>,A,Iterable<B>,A>() {
            @Override A of(Hom2<A, B, A> f, A z, Iterable<B> xs) {
                return foldl(f, z, xs);
            }
        };
    }
    public static <A> A joinl(final Monoid<A> f, Iterable<A> xs) {return foldl(f, f.id, xs);}
    public static <A> Hom2<Monoid<A>,Iterable<A>,A> joinl() {
        return new Hom2<Monoid<A>,Iterable<A>,A>() {@Override A of(Monoid<A> f, Iterable<A> xs) {
                return joinl(f, xs);
        }};
    }
    public static <A,B,C> ArrayList<C> zip(final Hom2<A,B,C> f, Iterable<A> xs, Iterable<B> ys) {
        final Iterator<A> ixs = xs.iterator();
        final Iterator<B> iys = ys.iterator();
        ArrayList<C> outs = new ArrayList<>();
        while (ixs.hasNext()&&iys.hasNext()) outs.add(f.of(ixs.next(), iys.next()));
        return outs;
    }
    public static <A,B,C> Hom3<Hom2<A,B,C>,Iterable<A>,Iterable<B>,ArrayList<C>> zip() {
        return new Hom3<Hom2<A,B,C>,Iterable<A>,Iterable<B>,ArrayList<C>>() {
            @Override ArrayList<C> of(Hom2<A, B, C> f, Iterable<A> xs, Iterable<B> ys) {
                return zip(f, xs, ys);
            }
        };
    }

    public static List<Integer> range(final int a, final int b) {
        return new ArrayList<Integer>() {{
            for(int i = a; i < b; i++) add(i);
        }};
    }
    public static List<Integer> range(final int b) {return range(0, b);}
}

---
layout: default
title: Reference
tagline: An invitation to aerospace control
description: Supplementary notes and other reference material
---

## Contents
{:.no_toc}

* This text will be replaced by a table of contents (excluding the above header) as an unordered list
{:toc}

---

## State space models

### What is a state space model?

A **state-space model** is a set of ordinary differential equations that can be written in this form:

$$
\dot{x} = Ax + Bu
$$

There are two variables:

* $x$ is the **state**
* $u$ is the **input**

Both variables are functions of time, so we could have written $x(t)$ and $u(t)$ --- but in general, we won't use that notation unless we need it. We use dots to indicate time derivatives, so $\dot{x}$ means $dx(t)/dt$.

The state and input may have more than one element --- that is to say, they may be vectors (represented by a matrix of numbers) and not scalars (represented by a single number). Suppose, in particular, that:

* the state $x$ has $n_x$ elements
* the input $u$ has $n_u$ elements

Then, we would represent them as column matrices:

$$
x =
\begin{bmatrix}
x_1 \\ \vdots \\ x_{n_x}
\end{bmatrix}
\qquad
u =
\begin{bmatrix}
u_1 \\ \vdots \\ u_{n_u}
\end{bmatrix}
$$

Taking the time derivative of a matrix is equivalent to taking the time derivative of each element of that matrix, so we would write

$$
\dot{x} =
\begin{bmatrix}
\dot{x}_1 \\ \vdots \\ \dot{x}_{n_x}
\end{bmatrix}
$$

The state-space model also has two constants: $A$ and $B$. If $x$ and $u$ are column matrices with (possibly) more than one element, then these constants have to be matrices as well:

* $A$ is a constant matrix of size $n_x \times n_x$
* $B$ is a constant matrix of size $n_x \times n_u$

The state space model has two key properties:

* it is **linear** because both $\dot{x}$ is a linear function of $x$ and $u$
* it is **time-invariant** because $A$ and $B$ are constant

<div class="alert alert-warning">
Other people use other symbols for both the variables and constants in a state space model. Indeed, we will sometimes use other symbols for these things as well. For example:

$$
\dot{z} = Ez + Fv
$$

This is also a "state space model," in which the state is $z$ and the input is $v$.
</div>


### How do I put a system in state space form?

Suppose we are given a description of a dynamic system. We would like to describe this same system with a state space model.

Remember that every state space model is linear. The equations of motion that describe a dynamic system are often nonlinear. So apparently, we will have to accept that state space models can only *approximate* some of the systems we want to describe.

We will use **linearization** to arrive at this approximation, in four steps.

**Step 1.** Rewrite the system as a set of first-order ordinary differential equations:

$$
\dot{m} = f(m,n)
$$

In this expression, the variables $m$ and $n$ are functions of time and can have more than one element --- in general, you should represent them as column matrices. As we said before, the function $f(\cdot)$ will often be nonlinear.

**Step 2.** Find an equilibrium point $m_{e}, n_{e}$ of the system by solving this equation:

$$
0 = f(m_{e},n_{e})
$$

A solution to this equation is called an equilibrium point because if

$$m = m_e \qquad n = n_e$$

then

$$\dot{m} = f(m_e, n_e) = 0$$

and so $m$ remains constant. That is to say, if the system reaches an equilibrium point, then it stays there. This is an important property! The goal of almost every controller we design in this course will be to make a system reach an equilibrium point quickly and reliably.

This equation may have no solutions, in which case no equilibrium point exists for the system. This is bad. We will ignore this situation for now.

This equation also may have many solutions, in which case you have to make a choice. A good choice is whatever equilibrium point you would like your system to achieve.

**Step 3.** Define the state and input as follows:

$$
x = m-m_{e}
\qquad
u = n-n_{e}
$$

Note that $x$ measures *error* --- the *difference* between $m$ and its equilibrium value. When error is zero, the system has reached equilibrium and is doing what we want. Apparently, with this way of defining the state and the input, "control design" means choosing $u$ so that $x$ goes to zero.

**Step 4.** Compute $A$ and $B$ as follows:

$$
A = \frac{\partial f}{\partial m}\biggr\rvert_{(m_{e},n_{e})}
\qquad
B = \frac{\partial f}{\partial n}\biggr\rvert_{(m_{e},n_{e})}
$$

<div class="alert alert-warning">
Recall that

$$
\frac{\partial f}{\partial m}\biggr\rvert_{(m_{e},n_{e})}
$$

is the <a href="https://en.wikipedia.org/wiki/Jacobian_matrix_and_determinant">Jacobian (i.e., matrix of partial derivatives)</a> of $f$ with respect to $m$, evaluated at the equilibrium point.
</div>

Why does this make any sense? Look again at the ODEs that describe the original system:

$$\dot{m} = f(m, n)$$

First, because

$$x = m - m_e$$

then

$$\dot{x} = \dot{m} - 0 = \dot{m}$$

So the *left*-hand side of the ODEs can simply be replaced with $\dot{x}$.

Now, suppose we want a linear approximation to the *right*-hand side of the ODEs --- the function $f(m, n)$. One way to find such an approximation is to take a [Taylor's series expansion](https://en.wikipedia.org/wiki/Taylor_series) about $m_e, n_e$ up to first order:

$$
\begin{aligned}
f(m, n)
&\approx f(m_e, n_e) + \frac{\partial f}{\partial m}\biggr\rvert_{(m_{e},n_{e})} \left( m - m_e \right) + \frac{\partial f}{\partial n}\biggr\rvert_{(m_{e},n_{e})} \left( n - n_e \right) \\
&= 0 + A x + B u \\
&= A x + B u
\end{aligned}
$$

There you have it: "$\dot{x} = Ax + Bu$" is a first-order (i.e., linear) approximation to "$\dot{m} = f(m, n)$".


#### Example (first-order)

Consider the system with dynamics that are described by the following equation of motion:

$$\dot{\omega} + 2 \omega = \tau$$

Let's apply our method to put this system in state space form.

We begin by rewriting it as a set of first-order ODEs. Lucky us, the system is already described by just one first-order ODE, so all we need to do is solve for $\dot{w}$:

$$
\dot{\omega} = f(\omega, \tau) = -2\omega + \tau
$$

Next, we find an equilibrium point by solving

$$0 = -2 \omega_e + \tau_e$$

In this case, there are many solutions. Suppose we pick this one:

$$\omega_e = 10 \qquad \tau_e = 20$$

Then, we define the state and input based on this choice of equilibrium point:

$$x = \omega - \omega_e \qquad u = \tau - \tau_e$$

Finally, we compute $A$ and $B$ by taking Jacobians (easy in this case because all the variables are scalars):

$$
\begin{aligned}
A &= \frac{\partial \left(-2\omega + \tau\right)}{\partial \omega}\biggr\rvert_{(\omega_{e},\tau_{e})} = -2 \\[1em]
B &= \frac{\partial \left(-2\omega + \tau\right)}{\partial \tau}\biggr\rvert_{(\omega_{e},\tau_{e})} = 1
\end{aligned}
$$

The resulting state space model is

$$
\begin{aligned}
\dot{x} &= Ax + Bu \\
&= -2x + u
\end{aligned}
$$

Note that the original system was linear, so there was no approximation here. We could probably have skipped most of these steps and written the system in state-space form by inspection. On the other hand, it is nice to know that the process of "linearization" still works even in this simple case.


#### Example (second-order)

Consider the system with dynamics that are described by the following equation of motion:

$$\ddot{q} + 3 \sin q = \tau$$

Let's apply our method to put this system in state-space form.

We begin by rewriting it as a set of first-order ODEs:

* We find the time-derivative of $q$ with highest order --- in this case, $\ddot{q}$, of order 2.
* We define *new variables* for each time-derivative of $q$ with lower order --- in this case, $\dot{q}$, the only time-derivative with order between 0 and 2. We might choose the following name for this time-derivative:

$$ v = \dot{q} $$

* We rewrite the original ODEs (in this case, just one) in terms of these new variables:

$$\dot{v} + 3\sin q = \tau$$

* We add one extra ODE for each new variable (in this case, just one extra) --- this is trivial, coming from the way we defined these new variables:

$$\dot{q} = v$$

* We collect the original ODEs and the extra ODEs together, if necessary solving for all of the time-derivatives (that's not necessary here):

$$
\begin{aligned}
\dot{q} &= v \\
\dot{v} &= -3\sin q + \tau
\end{aligned}
$$

* Finally, we rewrite our result in the form $\dot{m} = f(m, n)$ by collecting things in column matrices as follows:

$$\begin{bmatrix} \dot{q} \\ \dot{v} \end{bmatrix} = \begin{bmatrix} v \\ -3\sin q + \tau \end{bmatrix}$$

Note that, as desired, these rewritten ODEs have time derivatives that are at most of first order. Also note that all of these time-derivatives are on the left-hand side of the equations --- none appear on the right-hand side.

Next, we find an equilibrium point by solving

$$\begin{bmatrix} 0 \\ 0 \end{bmatrix} = \begin{bmatrix} v \\ -3\sin q + \tau \end{bmatrix}$$

There are many solutions. Suppose we pick this one:

In this case, there are many solutions. Suppose we pick this one:

$$q_e = \pi / 2 \qquad v_e = 0 \qquad \tau_e = 3$$

Then, we define the state and input based on this choice of equilibrium point:

$$x = \begin{bmatrix} q - q_e \\ v - v_e \end{bmatrix} \qquad u = \begin{bmatrix}\tau - \tau_e\end{bmatrix}$$

Finally, we compute $A$ and $B$ by taking Jacobians.

$$
\begin{aligned}
A
&= \frac{\partial f}{\partial \begin{bmatrix} q \\ v \end{bmatrix}}\biggr\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \left.\begin{bmatrix} \dfrac{\partial(v)}{\partial q} & \dfrac{\partial(v)}{\partial v} \\ \dfrac{\partial(-3\sin q + \tau)}{\partial q} & \dfrac{\partial(-3\sin q + \tau)}{\partial v} \end{bmatrix}\right\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \left.\begin{bmatrix} 0 & 1 \\ -3\cos q & 0 \end{bmatrix}\right\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix} \\[1em]
B
&= \frac{\partial f}{\partial \begin{bmatrix} \tau \end{bmatrix}}\biggr\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \left.\begin{bmatrix} \dfrac{\partial(v)}{\partial \tau} \\ \dfrac{\partial(-3\sin q + \tau)}{\partial \tau} \end{bmatrix}\right\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \left.\begin{bmatrix} 0 \\ 1 \end{bmatrix}\right\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \begin{bmatrix} 0 \\ 1 \end{bmatrix}
\end{aligned}
$$

The resulting state space model is

$$
\begin{aligned}
\dot{x} &= Ax + Bu \\
&= \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix}x + \begin{bmatrix} 0 \\ 1 \end{bmatrix}u
\end{aligned}
$$

The original system was nonlinear and the state space model is linear, so there *must* be some approximation here! As we will see, this approximation is good near the equilibrium point but can be very bad elsewhere.

# The matrix exponential function

## What is the matrix exponential?

The **matrix exponential** of a square matrix $M$ is the infinite series

$$
e^M = I + M + \frac{1}{2}M^2 + \frac{1}{6}M^3 + \dotsm = \sum_{k=0}^{\infty} \dfrac{1}{k !} M^k
$$

where $I$ is the identity matrix. This series converges for any square matrix $M$.


## How do I use the matrix exponential to solve a linear system?

The solution to the set of linear ODEs

$$ \dot{x} = Fx $$

with the initial condition

$$ x(t_0) = x_0 $$

is

$$ x(t) = e^{F (t - t_0)} x_0. $$

How do we know that this solution is correct? First, let's check that this solution satisfies the ODEs:

$$
\begin{aligned}
\dot{x}
&= \frac{d}{dt} \left( e^{F (t - t_0)} x_0 \right) \\
&= \frac{d}{dt} \left( \left( I + F(t-t_0) + \frac{1}{2} F^2(t-t_0)^2 + \frac{1}{6} F^2(t-t_0)^3 + \dotsm \right) x_0 \right) \\
&= \frac{d}{dt} \left( I + F(t-t_0) + \frac{1}{2} F^2(t-t_0)^2 + \frac{1}{6} F^3(t-t_0)^3 + \dotsm \right) x_0 \\
&= \left( \frac{d}{dt} \left( I \right) + \frac{d}{dt} \left( F(t-t_0) \right) + \frac{d}{dt} \left( \frac{1}{2} F^2(t-t_0)^2 \right) + \frac{d}{dt} \left( \frac{1}{6} F^3(t-t_0)^3 \right) \dotsm \right) x_0 \\
&= \left( 0 + F + F^2(t-t_0) + \frac{1}{2} F^3(t-t_0) + \dotsm \right) x_0 \\
&= F \left(I + F(t-t_0) + \frac{1}{2} F^2(t-t_0)^2 + \dotsm \right) x_0 \\
&= F e^{F(t-t_0)} x_0
\end{aligned}
$$

Apparently, it does. Second, let's check that this solution satisfies the initial condition:

$$ \begin{aligned} x(t_0) &= e^{F(t_0 - t_0)} x_0 \\ &= e^0 x_0 \\ &= I x_0 \\ &= x_0 \end{aligned} $$

Again, it does. (We might wonder if this is the *only* solution to the original ODEs --- it is, although a proof would require more work.)

## How do I use the matrix exponential to solve state space models?

Consider the state space model

$$ \dot{x} = Ax + Bu$$

This model does not look the same as

$$ \dot{x} = Fx $$

Indeed, without specifying $u$, we cannot solve for $x$ as a function of time. However, particular choices of $u$ allow us to simplify the state space model. For example, if we choose $u = 0$, then we can write

$$ \begin{aligned} \dot{x} &= Ax+Bu \\ &= Ax + B \cdot (0) \\ &= Ax + 0 \\ &= Ax \end{aligned} $$

and so we are right back at a linear system that can be solved with the matrix exponential. Another common choice of $u$ is

$$ u = -Kx $$

for some constant matrix $K$. (What would the size of $K$ have to be for us to define $u$ in this way?) This choice of $u$ is called **state feedback**, since the input depends on the state. If we plug this choice into our state space model, then we can write

$$
\begin{aligned}
\dot{x} &= Ax + Bu \\
&= Ax + B(-Kx) \\
&= (A - BK) x
\end{aligned}
$$

and so --- just like before --- we are right back at a linear system that can be solved with the matrix exponential. Although this result will get us a long way, we will see how to solve state space models for other choices of input later on.

<div class="alert alert-warning">
Given the state-space model

$$\dot{x} = Ax + Bu$$

it is standard to call the system

$$\dot{x} = Ax$$

that results from the application of zero input $u=0$ the <strong>open-loop system</strong>. Similarly, it is standard to call the system

$$\dot{x} = (A - BK) x$$

that results from the application of linear state feedback $u = -Kx$ the <strong>closed-loop system</strong>. Remember that "zero input" is not necessarily the same as "zero actuator commands." When <a href="#how-do-i-put-a-system-in-state-space-form">linearizing equations of motion to derive a state space model</a>, we defined

$$u = n - n_e$$

where $n$ was the set of actuator commands and $n_e$ was the value of these commands at equilibrium. So,

$$u = 0$$

actually means

$$n = n_e.$$

Similarly,

$$u = -Kx$$

actually means

$$n = n_e - Kx = n_e - K(m - m_e).$$

The term $n_e$ is typically referred to as <strong>feedforward</strong> and the term $-K(m - m_e)$ is typically referred to as <strong>feedback</strong>.
</div>


# Asymptotic stability

## What are eigenvalues and eigenvectors?

Consider a square matrix $F \in \mathbb{R}^{n \times n}$. If we can find a complex number $s \in \mathbb{C}$ and a non-zero, complex-valued vector $v \in \mathbb{C}^n$ that satisfy

$$ (s I - F) v = 0 $$

then we call $s$ an **eigenvalue** of $F$ and $v$ the corresponding **eigenvector** of $F$. If we wanted, we could rearrange this equation to put it in a form that might be more familiar:

$$ 0 = (s I - F) v = s v - F v \qquad\Rightarrow\qquad F v = s v. $$

One way to find eigenvalues is to solve the equation

$$ \det (s I - F) = 0 $$

where "$\det(\cdot)$" means taking a determinant. In general, this equation will be an $n$th-order polynomial in $s$, and so will have $n$ solutions --- we might call them $s_1, \dotsc, s_n$. To find an eigenvector that corresponds to each eigenvalue $s_i$, we solve

$$ F v_i = s_i v_i $$

for $v_i$. Note that there are many possible solutions to this equation and that eigenvectors are only unique up to a scale factor. In particular, for any real number $k \in \mathbb{R}$, we have

$$ \begin{aligned} F (k v_i) &= k (F v_i) \\ &= k (s v_i) \\ &= s (k v_i). \end{aligned}$$

Apparently, if $v_i$ is an eigenvector corresponding to $s_i$, then so is $k v_i$ for any $k \neq 0$. For this reason, algorithms to find eigenvectors typically *normalize* them to have unit length.

## How do I diagonalize a square matrix?

Suppose we have found the eigenvalues $s_1, \dotsc, s_n$ and eigenvectors $v_1, \dotsc, v_n$ of a square matrix $F\in\mathbb{R}^{n \times n}$. Define the matrix

$$V = \begin{bmatrix} v_1 & \dotsm & v_n \end{bmatrix}$$

with an eigenvector in each column, and also the matrix

$$\text{diag} (s_1, \dotsc, s_n) = \begin{bmatrix} s_1 & 0 & \dotsm & 0 \\ 0 & s_2 & \dotsm & 0 \\ \vdots & \vdots & \ddots & \vdots \\ 0 & 0 & \dotsm & s_n \end{bmatrix}$$

with the eigenvalues along the diagonal.

Two things are true.

First, the following equality holds:

$$F V = V \text{diag} (s_1, \dotsc, s_n) $$

You could easily verify this result for yourself.

Second, if $s_1, \dotsc, s_n$ are all *distinct* (i.e., if no two eigenvalues are the same), then the matrix $V$ is *invertible*. This result is harder to verify --- it has to do with the fact that if the eigenvalues are distinct then the eigenvectors are linearly independent.

The key consequence of $V$ being invertible is that we can solve the above equality to write:

$$\text{diag} (s_1, \dotsc, s_n) = V^{-1} F V.$$

In this case --- if all eigenvalues are distinct and so the matrix of eigenvectors is invertible --- we say that $F$ is **diagonalizable**. The process of "diagonalizing $F$" is finding the matrix $V$.

## What is the matrix exponential of a diagonal matrix?

It is easy to find the matrix exponential of a diagonal matrix, starting from [the definition](#what-is-the-matrix-exponential):

$$
\begin{align*}
e^{\text{diag} (s_1, \dotsc, s_n)t}
&= I + \text{diag} (s_1, \dotsc, s_n)t + \frac{1}{2}\left( \text{diag} (s_1, \dotsc, s_n) t \right)^2 + \dotsm \\
&=
\begin{bmatrix} 1 & 0 & \dotsm & 0 \\ 0 & 1 & \dotsm & 0 \\ \vdots & \vdots & \ddots & \vdots \\ 0 & 0 & \dotsm & 1 \end{bmatrix}
+
\begin{bmatrix} s_1t & 0 & \dotsm & 0 \\ 0 & s_2t & \dotsm & 0 \\ \vdots & \vdots & \ddots & \vdots \\ 0 & 0 & \dotsm & s_nt \end{bmatrix}
+
\begin{bmatrix} (s_1t)^2/2 & 0 & \dotsm & 0 \\ 0 & (s_2t)^2/2 & \dotsm & 0 \\ \vdots & \vdots & \ddots & \vdots \\ 0 & 0 & \dotsm & (s_nt)^2/2 \end{bmatrix}
+ \dotsm \\
&= \begin{bmatrix} 1 + s_1t + (s_1t)^2/2 + \dotsm & 0 & \dotsm & 0 \\ 0 & 1 + s_2t + (s_2t)^2/2 + \dotsm & \dotsm & 0 \\ \vdots & \vdots & \ddots & \vdots \\ 0 & 0 & \dotsm & 1 + s_nt + (s_nt)^2/2 + \dotsm \end{bmatrix} \\
&= \begin{bmatrix} e^{s_1t} & 0 & \dotsm & 0 \\ 0 & e^{s_2t} & \dotsm & 0 \\ \vdots & \vdots & \ddots & \vdots \\ 0 & 0 & \dotsm & e^{s_nt} \end{bmatrix}
\end{align*}
$$

## What is the solution to a linear system that is diagonalizable?

[We have seen](#how-do-i-use-the-matrix-exponential-to-solve-a-linear-system) that the solution to

$$\dot{x} = Fx$$

with the initial condition

$$x(0) = x_0$$

is

$$x(t) = e^{Ft}x_0.$$

Suppose $F$ is a diagonalizable matrix, so that

$$\text{diag} (s_1, \dotsc, s_n) = V^{-1} F V$$

where

$$\text{diag} (s_1, \dotsc, s_n)$$

is a diagonal matrix that contains the eigenvalues of $F$ and where

$$V = \begin{bmatrix} v_1 & \dotsm & v_n \end{bmatrix}$$

is a matrix of the corresponding eigenvectors. Then, applying [the definition of matrix exponential](#what-is-the-matrix-exponential) again, we have

$$
\begin{align*}
e^{Ft}x_0
&= e^{V \text{diag} (s_1, \dotsc, s_n) V^{-1}t}x_0 \\
&= \left( I + V \text{diag} (s_1, \dotsc, s_n) V^{-1}t + \frac{1}{2}\left( V \text{diag} (s_1, \dotsc, s_n) V^{-1}t \right)^2 + \dotsm\right) x_0 \\
&= V \left( I + \text{diag} (s_1, \dotsc, s_n) t + \frac{1}{2} \left( \text{diag} (s_1, \dotsc, s_n)t \right)^2 + \dotsm\right) V^{-1}x_0 \\
&= V e^{\text{diag} (s_1, \dotsc, s_n)t}V^{-1}x_0 \\
&= V \begin{bmatrix} e^{s_1t} & 0 & \dotsm & 0 \\ 0 & e^{s_2t} & \dotsm & 0 \\ \vdots & \vdots & \ddots & \vdots \\ 0 & 0 & \dotsm & e^{s_nt} \end{bmatrix} V^{-1}x_0
\end{align*}
$$

where the last step comes from what we just found out about the [matrix exponential of a diagonal matrix](#what-is-the-matrix-exponential-of-a-diagonal-matrix). In this expression, the terms $V$, $V^{-1}$, and $x_0$ are constant. The only terms that depend on $t$, in fact, are the *scalar* exponential functions

$$e^{s_1t}, \dotsc, e^{s_nt}$$

that appear in the diagonal of

$$e^{\text{diag} (s_1, \dotsc, s_n)t}.$$

Therefore, we can infer the behavior of $x(t)$ based entirely on these scalar exponential functions. In particular, suppose that each eigenvalue $s_i$ — a complex number — has real part $a_i$ and imaginary part $b_i$, or in other words that

$$s_i = a_i + jb_i.$$

[Euler's formula](https://en.wikipedia.org/wiki/Euler%27s_formula) tells us that

$$e^{s_it} = e^{(a_i+jb_i)t} = e^{a_it}\left(\cos(b_it) + j\sin(b_it)\right).$$

Apparently, as time gets large, one of three things is true about each of these terms:

* if $a_i > 0$, then $e^{(a_i+jb_i)t}$ grows quickly
* if $a_i = 0$, then $e^{(a_i+jb_i)t}$ is constant ($b_i=0$) or is sinusoidal with constant magnitude ($b_i \neq 0$)
* if $a_i < 0$, then $e^{(a_i+jb_i)t}$ decays quickly to zero

It is possible to show that (more or less) the same result holds for *any* system $\dot{x}=Fx$, not only ones for which $F$ is diagonalizable. This takes more work, and involves the transformation of $F$ into [Jordan normal form](https://en.wikipedia.org/wiki/Jordan_normal_form) rather than into a diagonal matrix. We would discover that the terms that depend on $t$ all look like

$$t^me^{s_it}$$

where $m$ is an integer that is at most the multiplicity of the eigenvalue $s_i$. Since $e^{a_it}$ increases or decreases a lot faster than $t^m$, then the same three things we listed above would be true of each term in $x(t)$, just like before.

See [the reference textbook](https://fbswiki.org/) for details.

## When is a linear system asymptotically stable?

The system

$$\dot{x} = Fx$$

is called **asymptotically stable** if $x(t) \rightarrow 0$ as $t \rightarrow \infty$, starting from any initial condition $x(t_0) = x_0.$

Based on our observations about [the solution to linear systems that are diagonalizable](#what-is-the-solution-to-a-linear-system-that-is-diagonalizable), we can state the following important result:

<div class="alert alert-warning">
The system

$$\dot{x} = Fx$$

is asymptotically stable if and only if all eigenvalues of $F$ have negative real part.
</div>

In particular, we now have a test for whether or not a controller "works." Suppose we apply linear state feedback

$$u = -Kx$$

to the state space system

$$\dot{x} = Ax + Bu$$

so that

$$\dot{x} = (A - BK)x.$$

The controller "works" when this system is asymptotically stable, i.e., when $x$ goes to zero as time gets large. We now know, therefore, that the controller "works" when all eigenvalues of $A - BK$ have negative real part.

We may not have a systematic way of *finding* a matrix $K$ to make the closed-loop system stable yet, but we certainly do have a systematic way now of deciding whether or not a *given* matrix $K$ makes the closed-loop system stable.


# Optimization and Optimal Control

These notes were originally written by T. Bretl and were transcribed for this reference page by S. Bout.

## Optimization

The following thing is called an *optimization problem*:

$$\begin{align*}
\mathop{\mathrm{minimize}}_{u} \qquad u^{2}-2u+3
\end{align*}$$

The solution to this problem is the value of $u$ that makes $u^{2}-2u+3$ as small as
possible.


-   We know that we are supposed to choose a value of $u$ because "$u$"
    appears underneath the "minimize" statement. We call $u$ the
    *decision variable*.

-   We know that we are supposed to minimize $u^{2}-2u+3$ because
    "$u^{2}-2u+3$" appears to the right of the "minimize" statement. We
    call $u^{2}-2u+3$ the *cost function*.

In particular, the solution to this problem is $u=1$. There are at least
two different ways to arrive at this result:

-   We could plot the cost function. It is clear from the plot that the
    minimum is at $u=1$.\
    ![image](./images/optim01.jpg)

-   We could apply the first derivative test. We compute the first
    derivative:

    $$\begin{aligned}
    \frac{d}{du}  (u^{2}-2u+3) = 2 u - 2
    \end{aligned}$$

    Then, we set the first derivative equal to zero and solve for $u$:

    $$
    \begin{align*}
    2u-2 = 0 \qquad \Rightarrow \qquad u=1
    \end{align*}$$

    Values of $u$ that satisfy
    the first derivative test are only "candidates" for
    optimality---they could be maxima instead of minima, or could be
    only one of many minima. We'll ignore this distinction for now.
    Here's a plot of the cost function and of it's derivative. Note
    that, clearly, the derivative is equal to zero when the cost
    function is minimized:\
    ![image](./images/optim03.jpg)

In general, we write optimization problems like this:

$$
\begin{align*}
\mathop{\mathrm{minimize}}_{u} \qquad g(u)
\end{align*}$$

Again, $u$ is the
decision variable and $g(u)$ is the cost function. In the previous
example:

$$
\begin{align*}
g(u)=u^{2}-2u+3
\end{align*}$$

Here is another example:

$$
\begin{align*}
\mathop{\mathrm{minimize}}_{u_{1},u_{2}} \qquad u_{1}^{2}+3u_{2}^{2}-2u_{1}u_{2}+2u_{1}+2u_{2}+6
\end{align*}$$

The solution to this problem is the value of both $u_{1}$ and $u_{2}$
that, together, make $u_{1}^{2}+3u_{2}^{2}-2u_{1}u_{2}+2u_{1}+2u_{2}+6$
as small as possible. There are two differences between this
optimization problem and the previous one. First, there is a different
cost function:

$$
\begin{align*}
g(u_{1},u_{2}) = u_{1}^{2}+3u_{2}^{2}-2u_{1}u_{2}+2u_{1}+2u_{2}+6
\end{align*}$$

Second, there are two decision variables instead of one. But again,
there are at least two ways of finding the solution to this problem:

-   We could plot the cost function. The plot is now 3D---the "x" and
    "y" axes are $u_{1}$ and $u_{2}$, and the "z" axis is
    $g(u_{1},u_{2})$. The shape of the plot is a bowl. It's hard to tell
    where the minimum is from looking at the bowl, so I've also plotted
    contours of the cost function underneath. "Contours" are like the
    lines on a topographic map. From the contours, it looks like the
    minimum is at $(u_{1},u_{2})=(-2,-1)$.\
    ![image](./images/optim04.jpg)

-   We could apply the first derivative test. We compute the partial
    derivative of $g(u_{1},u_{2})$ with respect to both $u_{1}$ and
    $u_{2}$:

    $$\begin{aligned}
    \frac{\partial}{\partial u_{1}} g(u_{1},u_{2}) &= 2u_{1}-2u_{2}+2 \\
    \frac{\partial}{\partial u_{2}} g(u_{1},u_{2}) &= 6u_{2}-2u_{1}+2\end{aligned}$$

    Then, we set both partial derivatives equal to zero and solve for
    $u_{1}$ and $u_{2}$:

    $$
    \begin{align*}
    \begin{split}
    2u_{1}-2u_{2}+2 &= 0\\
    6u_{2}-2u_{1}+2 &= 0
    \end{split}
    \qquad \Rightarrow \qquad
    (u_{1},u_{2}) = (-2,-1)
    \end{align*}$$

    As before, we would have to apply a
    further test in order to verify that this choice of $(u_{1},u_{2})$
    is actually a minimum. But it is certainly consistent with what we
    observed above. Here is a plot of each partial derivative as a
    function of $u_{1}$ and $u_{2}$. The shape of each plot is a plane
    (i.e., a flat surface). Both planes are zero at $(-2,-1)$:\
    ![image](./images/optim05.jpg)

An equivalent way of stating this same optimization problem would have
been as follows:

$$
\begin{align*}
\mathop{\mathrm{minimize}}_{u_{1},u_{2}} \qquad \begin{bmatrix} u_{1} \\ u_{2} \end{bmatrix}^{T} \begin{bmatrix} 1 & -1 \\ -1 & 3 \end{bmatrix} \begin{bmatrix} u_{1} \\ u_{2} \end{bmatrix} + \begin{bmatrix} 2 \\ 2 \end{bmatrix}^{T} \begin{bmatrix} u_{1} \\ u_{2} \end{bmatrix}+6
\end{align*}$$

You can check that the cost function shown above is the same as the cost
function we saw before (e.g., by multiplying it out). We could have gone
farther and stated the problem as follows:

$$
\begin{align*}
\mathop{\mathrm{minimize}}_{u} \qquad u^{T} \begin{bmatrix} 1 & -1 \\ -1 & 3 \end{bmatrix} u + \begin{bmatrix} 2 \\ 2 \end{bmatrix}^{T} u+6
\end{align*}$$

We have returned to having just one decision variable $u$, as in the
first example, but this variable is now a $2\times 1$ matrix---i.e., it
has two elements, which we would normally write as $u_{1}$ and $u_{2}$.
The point here is that the "decision variable" in an optimization
problem can be a variety of different things: a scalar, a vector (i.e.,
an $n\times 1$ matrix), and---as we will see---even a function of time.
Before proceeding, however, let's look at one more example of an
optimization problem:

$$\begin{aligned}
\mathop{\mathrm{minimize}}_{u,x} &\qquad u^{2}+3x^{2}-2ux+2u+2x+6 \\
\text{subject to} &\qquad u+x=3\end{aligned}$$

This example is exactly
the same as the previous example, except that the two decision variables
(now renamed $u$ and $x$) are subject to a constraint: $u+x=3$. We are
no longer free to choose $u$ and $x$ arbitrarily. We are restricted to
choices that satisfy the constraint. The solution to this optimization
problem is the value $(u,x)$ that minimizes the cost function, chosen
from among all values $(u,x)$ that satisfy the constraint. Again, there
are a variety of ways to solve this problem. One way is to eliminate the
constraint. First, we solve the constraint equation:

$$
\begin{align*}
u+x=3 \qquad\Rightarrow\qquad x = 3-u
\end{align*}$$

Then, we plug this result into
the cost function:

$$\begin{aligned}
u^{2}+3x^{2}-2ux+2u+2x+6
&= u^{2}+3(3-u)^{2}-2u(3-u)+2u+2(3-u)+6 \\
&= 6u^{2}-24u+39\end{aligned}$$

By doing so, we have shown that solving
the constrained optimization problem

$$\begin{aligned}
\mathop{\mathrm{minimize}}_{u,x} &\qquad u^{2}+3x^{2}-2ux+2u+2x+6 \\
\text{subject to} &\qquad u+x=3\end{aligned}$$

is equivalent to solving
the unconstrained optimization problem

$$
\begin{align*}
\mathop{\mathrm{minimize}}_{u} \qquad 6u^{2}-24u+39
\end{align*}$$

and then taking
$x=3-u$. We can do so easily by taking the first derivative and setting
it equal to zero, as we did in the first example:

$$
\begin{align*}
0 = \frac{d}{du} \left(6u^{2}-24u+39\right) = 12u-24 \qquad\Rightarrow\qquad u = 2 \qquad\Rightarrow\qquad x = 3-u= 1
\end{align*}$$

The point here was not to show how to solve constrained optimization
problems in general, but rather to identify the different parts of a
problem of this type. As a quick note, you will sometimes see the
example optimization problem we've been considering written as

$$\begin{aligned}
\mathop{\mathrm{minimize}}_{u} &\qquad u^{2}+3x^{2}-2ux+2u+2x+6 \\
\text{subject to} &\qquad u+x=3\end{aligned}$$

The meaning is exactly
the same, but $x$ isn't listed as one of the decision variables under
"minimize." The idea here is that $x$ is an "extra variable" that we
don't really care about. This optimization problem is trying to say the
following:

> "Among all choices of $u$ for which there exists an $x$ satisfying
> $u+x=3$, find the one that minimizes $u^{2}+3x^{2}-2ux+2u+2x+6$."

## Minimum vs. Minimizer {#sec:minimum}

We have seen three example problems. In each case, we were looking for
the minimizer, i.e., the choice of decision variable that made the cost
function as small as possible:

-   The solution to

    $$
    \begin{align*}
    \mathop{\mathrm{minimize}}_{u} \qquad u^{2}-2u+3
    \end{align*}
    $$

    was $u=1$.

-   The solution to

    $$
    \begin{align*}
    \mathop{\mathrm{minimize}}_{u_{1},u_{2}} \qquad u_{1}^{2}+3u_{2}^{2}-2u_{1}u_{2}+2u_{1}+2u_{2}+6
    \end{align*}$$

    was $(u_{1},u_{2})=(-2,-1)$.

-   The solution to

    $$\begin{aligned}
    \mathop{\mathrm{minimize}}_{u} &\qquad u^{2}+3x^{2}-2ux+2u+2x+6 \\
    \text{subject to} &\qquad u+x=3\end{aligned}$$

    was $(u,x)=(2,1)$.

It is sometimes useful to focus on the minimum instead of on the
minimizer, i.e., what the "smallest value" was that we were able to
achieve. When focusing on the minimum, we often use the following "set
notation" instead:

-   The problem

    $$
    \begin{align*}
    \mathop{\mathrm{minimize}}_{u} \qquad u^{2}-2u+3
    \end{align*}$$

    is rewritten

    $$
    \begin{align*}
    \mathop{\mathrm{minimum}}_{u} \left\{ u^{2}-2u+3 \right\}.
    \end{align*}$$

    The meaning is---find the minimum value of $u^{2}-2u+3$ over all choices of $u$. The solution to this problem can be found by plugging in what we already know is the minimizer, $u=1$. In particular, we find that the solution is $2$.

-   The problem

    $$
    \begin{align*}
    \mathop{\mathrm{minimize}}_{u_{1},u_{2}} \qquad u_{1}^{2}+3u_{2}^{2}-2u_{1}u_{2}+2u_{1}+2u_{2}+6
    \end{align*}$$

    is rewritten

    $$
    \begin{align*}
    \mathop{\mathrm{minimum}}_{u_{1},u_{2}} \left\{ u_{1}^{2}+3u_{2}^{2}-2u_{1}u_{2}+2u_{1}+2u_{2}+6 \right\}.
    \end{align*}$$

    Again, the meaning is---find the minimum value of $u_{1}^{2}+3u_{2}^{2}-2u_{1}u_{2}+2u_{1}+2u_{2}+6$ over all choices of $u_{1}$ and $u_{2}$. We plug in what we already know is the minimizer $(u_{1},u_{2})=(-2,-1)$ to find the solution---it is $3$.

-   The problem

    $$\begin{aligned}
    \mathop{\mathrm{minimize}}_{u} &\qquad u^{2}+3x^{2}-2ux+2u+2x+6 \\
    \text{subject to} &\qquad u+x=3\end{aligned}$$

    is rewritten

    $$
    \begin{align*}
    \mathop{\mathrm{minimum}}_{u} \left\{ u^{2}+3x^{2}-2ux+2u+2x+6 \;\colon\; u+x=3  \right\}.
    \end{align*}$$

    And again, the meaning is---find the minimum value of $u^{2}+3x^{2}-2ux+2u+2x+6$ over all choices of $u$ for which there exists $x$ satisfying $u+x=3$. Plug in the known minimizer, $(u,x)=(2,1)$, and we find that the solution is 15.

The important thing here is to understand the notation and to understand
the difference between a "minimum" and a "minimizer."

## Optimal Control

### Statement of the problem

The following thing is called an *optimal control problem*:

$$
\begin{align*}
\tag{1}
\mathop{\mathrm{minimize}}_{u_{[t_{0},t_{1}]}} &\qquad h(x(t_{1})) + \int_{t_{0}}^{t_{1}}g(x(t),u(t))dt \\
\text{subject to} &\qquad \frac{dx(t)}{dt} = f(x(t),u(t)), \quad x(t_{0})=x_{0}
\end{align*}$$

Let's try to understand what it means.

-   The statement

    $$\begin{align*}
    \mathop{\mathrm{minimize}}_{u_{[t_{0},t_{1}]}}
    \end{align*}$$

    says that we are being asked to choose an input trajectory $u$ that
    minimizes something. Unlike in the optimization problems we saw
    before, the decision variable $u$ in this problem is a function of
    time. The notation $u_{[t_{0},t_{1}]}$ is one way of indicating
    this. Given an initial time $t_{0}$ and a final time $t_{1}$, we are
    being asked to choose the value of $u(t)$ at all times in between,
    i.e., for all $t\in[t_{0},t_{1}]$.

-   The statement

    $$
    \begin{align*}
    \frac{dx(t)}{dt} = f(x(t),u(t)), \quad x(t_{0})=x_{0}
    \end{align*}$$

    is a
    constraint. It implies that we are restricted to choices of $u$ for
    which there exists an $x$ satisfying a given initial condition

    $$
    \begin{align*}
    x(t_{0}) = x_{0}
    \end{align*}$$

    and satisfying the ordinary differential
    equation

    $$
    \begin{align*}
    \frac{dx(t)}{dt} = f(x(t),u(t)).
    \end{align*}$$

    One example of an
    ordinary differential equation that looks like this is our usual
    description of a system in state-space form:

    $$
    \begin{align*}
    \dot{x} = Ax+Bu,
    \end{align*}$$

-   The statement

    $$
    \begin{align*}
    h(x(t_{1})) + \int_{t_{0}}^{t_{1}}g(x(t),u(t))dt
    \end{align*}$$

    says what we are trying to minimize---it is the cost function in
    this problem. Notice that the cost function depends on both $x$ and
    $u$. Part of it---$g(\cdot)$---is integrated (i.e., "added up") over
    time. Part of it---$h(\cdot)$---is applied only at the final time.
    One example of a cost function that looks like this is

    $$\begin{align*}
    x(t_{1})^{T}Mx(t_{1}) + \int_{t_{0}}^{t_{1}} \left( x(t)^{T}Qx(t)+u(t)^{T}Ru(t) \right) dt
    \end{align*}$$

### The HJB equation (our new "first-derivative test")

As usual, there are a variety of ways to solve an optimal control
problem. One way is by application of what is called the
*Hamilton-Jacobi-Bellman Equation*, or "HJB." This equation is to
optimal control what the first-derivative test is to optimization. To
derive it, we will first rewrite the optimal control problem in "minimum
form" (see "Minimum vs Minimizer Section"):

$$
\begin{align*}
\mathop{\mathrm{minimum}}_{u_{[t_{0},t_{1}]}} \left\{ h(x(t_{1})) + \int_{t_{0}}^{t_{1}}g(x(t),u(t))dt
\;\colon\; \frac{dx(t)}{dt} = f(x(t),u(t)), \quad x(t_{0})=x_{0} \right\}
\end{align*}$$

Nothing has changed here, we're just asking for the minimum and not the
minimizer. Next, rather than solve this problem outright, we will first
state a slightly different problem:

$$\begin{align*}
\tag{2}
\mathop{\mathrm{minimum}}_{\bar{u}_{[t,t_{1}]}} \left\{ h(\bar{x}(t_{1})) + \int_{t}^{t_{1}}g(\bar{x}(s),\bar{u}(s))ds
\;\colon\; \frac{d\bar{x}(s)}{ds} = f(\bar{x}(s),\bar{u}(s)), \quad \bar{x}(t)=x \right\}
\end{align*}$$

The two changes that I made to go from the original problem to this one
are:

-   Make the initial time arbitrary (calling it $t$ instead of $t_{0}$).

-   Make the initial state arbitrary (calling it $x$ instead of
$x_{0}$).

I also made three changes in notation. First, I switched from $x$ to
$\bar{x}$ to avoid getting confused between $x$ as initial condition and
$\bar{x}$ as state trajectory. Second, I switched from $u$ to $\bar{u}$
to be consistent with the switch from $x$ to $\bar{x}$. Third, I
switched from calling time $t$ to calling time $s$ to avoid getting
confused with my use of $t$ as a name for the initial time.

You should think of the problem (2)
as a function itself. In goes an initial time $t$ and an initial state
$x$, and out comes a minimum value. We can make this explicit by writing

$$\begin{align*}
\tag{3}
v(t,x) = \mathop{\mathrm{minimum}}_{\bar{u}_{[t,t_{1}]}} \left\{ h(\bar{x}(t_{1})) + \int_{t}^{t_{1}}g(\bar{x}(s),\bar{u}(s))ds
\;\colon\; \frac{d\bar{x}(s)}{ds} = f(\bar{x}(s),\bar{u}(s)), \quad \bar{x}(t)=x \right\}
\end{align*}$$

We call $v(t,x)$ the *value function*. Notice that $v(t_{0},x_{0})$ is
the solution to the original optimal control problem that we wanted to
solve---the one where the initial time is $t_{0}$ and the initial state
is $x_{0}$. More importantly, notice that $v(t,x)$ satisfies the
following recursion:

$$
\begin{align*}
\tag{4}
v(t,x) = \mathop{\mathrm{minimum}}_{\bar{u}_{[t,t+\Delta t]}} \{ v(t+\Delta t, \bar{x}(t+\Delta t)) + \int_{t}^{t+\Delta t}g(\bar{x}(s),\bar{u}(s))ds\colon \\
\qquad\qquad\qquad\qquad\qquad\qquad\qquad \frac{d\bar{x}(s)}{ds} = f(\bar{x}(s),\bar{u}(s)), \quad \bar{x}(t)=x\}
\end{align*}$$

The reason this equation is called a "recursion" is that
it expresses the function $v$ in terms of itself. In particular, it
splits the optimal control problem into two parts. The first part is
from time $t$ to time $t+\Delta t$. The second part is from time
$t+\Delta t$ to time $t_{1}$. The recursion says that the minimum value
$v(t,x)$ is the sum of the cost

$$
\begin{align*}
\mathop{\mathrm{minimum}}_{\bar{u}_{[t,t+\Delta t]}} \left\{ \int_{t}^{t+\Delta t}g(\bar{x}(s),\bar{u}(s))ds
\;\colon\; \frac{d\bar{x}(s)}{ds} = f(\bar{x}(s),\bar{u}(s)), \quad \bar{x}(t)=x \right\}
\end{align*}$$

from the first part and the cost

$$
\begin{align*}
\mathop{\mathrm{minimum}}_{\bar{u}_{[t+\Delta t,t_{1}]}} \left\{ h(\bar{x}(t_{1})) + \int_{t+\Delta t}^{t_{1}}g(\bar{x}(t),\bar{u}(t))dt
\;\colon\; \frac{d\bar{x}(s)}{ds} = f(\bar{x}(s),\bar{u}(s)), \quad \bar{x}(t+\Delta t)=\text{blah} \right\}
\end{align*}$$

from the second part (where "$\text{blah}$" is whatever the state turns
out to be, starting at time $t$ from start $x$ and applying the input
$u_{[t,t+\Delta t]}$), which we recognize as the definition of

$$
\begin{align*}
v\left(t+\Delta t, \bar{x}(t+\Delta t)\right).
\end{align*}$$

We now proceed to
approximate the terms in (4) by first-order series expansions. In
particular, we have

$$\begin{aligned}
v\left(t+\Delta t, \bar{x}(t+\Delta t)\right)
&\approx v\left(t+\Delta t, \bar{x}(t) + \frac{d\bar{x}(t)}{dt}\Delta t\right) \\
&= v\left(t+\Delta t, x + f(x,\bar{u}(t))\Delta t\right) \\
&\approx v(t,x)+\frac{\partial v(t,x)}{\partial t} \Delta t + \frac{\partial v(t,x)}{\partial x} f(x,\bar{u}(t))\Delta t\end{aligned}$$

and we also have

$$\begin{aligned}
\int_{t}^{t+\Delta t}g(\bar{x}(s),\bar{u}(s))ds
&\approx g(\bar{x}(t),\bar{u}(t)) \Delta t \\
&= g(x,\bar{u}(t))\Delta t.\end{aligned}$$

If we plug both of these
into (4), we find

$$\begin{align*}
v(t,x) = \mathop{\mathrm{minimum}}_{\bar{u}_{[t,t+\Delta t]}} \{ v(t+\Delta t, \bar{x}(t+\Delta t)) + \int_{t}^{t+\Delta t}g(\bar{x}(s),\bar{u}(s))ds\colon \\
\qquad\qquad\qquad\qquad\qquad\qquad\qquad \frac{d\bar{x}(s)}{ds} = f(\bar{x}(s),\bar{u}(s)), \quad \bar{x}(t)=x\} \\
= \mathop{\mathrm{minimum}}_{\bar{u}_{[t,t+\Delta t]}} \{
v(t,x)+\frac{\partial v(t,x)}{\partial t} \Delta t + \frac{\partial v(t,x)}{\partial x} f(x,\bar{u}(t))\Delta t + g(x,\bar{u}(t))\Delta t \colon \\
\qquad\qquad\qquad\qquad\qquad\qquad\qquad \frac{d\bar{x}(s)}{ds} = f(\bar{x}(s),\bar{u}(s)), \quad \bar{x}(t)=x\}.
\end{align*}$$

Notice that nothing inside the minimum depends on anything other than
$t$, $x$, and $\bar{u}(t)$. So we can drop the constraint and make
$\bar{u}(t)$ the only decision variable. In fact, we might as well
replace $\bar{u}(t)$ simply by "$u$" since we only care about the input
at a single instant in time:

$$
\begin{align*}
v(t,x) =  \mathop{\mathrm{minimum}}_{u} \{ v(t,x)+\frac{\partial v(t,x)}{\partial t} \Delta t + \frac{\partial v(t,x)}{\partial x} f(x,u)\Delta t + g(x,u)\Delta t \}.
\end{align*}
$$

Also, notice that

$$
\begin{align*}
v(t,x)+\frac{\partial v(t,x)}{\partial t} \Delta t
\end{align*}$$

does not depend
on $u$, so it can be brought out of the minimum:

$$
\begin{align*}
v(t,x) = v(t,x)+\frac{\partial v(t,x)}{\partial t} \Delta t + \mathop{\mathrm{minimum}}_{u} \{\frac{\partial v(t,x)}{\partial x} f(x,u)\Delta t + g(x,\bar{u}(t))\Delta t\}.
\end{align*}
$$

To simplify further, we can
subtract $v(t,x)$ from both sides, then divide everything by $\Delta t$:

$$
\begin{align*}
\tag{5}
0 = \frac{\partial v(t,x)}{\partial t} + \mathop{\mathrm{minimum}}_{u} \{
\frac{\partial v(t,x)}{\partial x} f(x,u) + g(x,u) \}.
\end{align*}
$$

The equation is called the *Hamilton-Jacobi-Bellman Equation*, or
simply the HJB equation. As you can see, it is a partial differential
equation, so it needs a boundary condition. This is easy to obtain. In particular, going all the way back to the definition (3), we find that

$$
\begin{align*}
\tag{6}
v(t_{1},x) = h(x).
\end{align*}$$

 The importance of HJB is that if you can find a
solution to (5)-(6))---that is, if you can find a function $v(t,x)$ that satisfies the partial differential equation (5) and the
boundary condition (6)---then the minimizer $u$ in (5), evaluated
at every time $t\in[t_{0},t_{1}]$, is the solution to the optimal
control problem (1). You might object that (5) "came out
of nowhere." First of all, it didn't. We derived it just now, from
scratch. Second of all, where did the first-derivative test come from?
Could you derive that from scratch? (Do you, every time you need to use
it? Or do you just use it?)

### Solution approach {#secApproach}

The optimal control problem

$$\begin{aligned}
\mathop{\mathrm{minimize}}_{u_{[t_{0},t_{1}]}} &\qquad h(x(t_{1})) + \int_{t_{0}}^{t_{1}}g(x(t),u(t))dt \\
\text{subject to} &\qquad \frac{dx(t)}{dt} = f(x(t),u(t)), \quad x(t_{0})=x_{0}\end{aligned}$$

can be solved in two steps:

-   Find $v$:

    $$\begin{align*}
    0 = \frac{\partial v(t,x)}{\partial t} + \mathop{\mathrm{minimum}}_{u} \{
    \frac{\partial v(t,x)}{\partial x} f(x,u) + g(x,u) \},
    \qquad
    v(t_{1},x) = h(x)
    \end{align*}$$

-   Find $u$:

    $$
    \begin{align*}
    u(t) = \mathop{\mathrm{minimize}}_{u}\; \frac{\partial v(t,x)}{\partial x} f(x,u) + g(x,u)
    \qquad\qquad
    \text{for all } t\in[t_{0},t_{1}]
    \end{align*}$$

## LQR

### Statement of the problem {#secStatement}

Here is the *linear quadratic regulator (LQR)* problem:

$$\begin{aligned}
\mathop{\mathrm{minimize}}_{u_{[t_{0},t_{1}]}} &\qquad x(t_{1})^{T}Mx(t_{1}) + \int_{t_{0}}^{t_{1}}\left( x(t)^{T}Qx(t)+u(t)^{T}Ru(t)\right)dt \\
\text{subject to} &\qquad \dot{x}(t) = Ax(t)+Bu(t), \quad x(t_{0})=x_{0}\end{aligned}$$

It is an optimal control problem---if you define

$$
\begin{align*}
f(x,u) = Ax+Bu \qquad\qquad g(x,u) = x^{T}Qx+u^{T}Ru \qquad\qquad h(x) = x^{T}Mx
\end{align*}
$$

then you see that this problem has the same form as (1). It is
called "linear" because the dynamics are those of a linear (state space)
system. It is called "quadratic" because the cost is quadratic (i.e.,
polynomial of order at most two) in $x$ and $u$. It is called a
"regulator" because the result of solving it is to keep $x$ close to
zero (i.e., to keep errors small).

The matrices $Q$, $R$, and $M$ are parameters that can be used to trade
off error (non-zero states) with effort (non-zero inputs). These
matrices have to be symmetric ($Q=Q^{T}$, etc.), have to be the right
size, and also have to satisfy the following conditions in order for the
LQR problem to have a solution:

$$
\begin{align*}
Q \geq 0 \qquad\qquad R>0 \qquad\qquad M\geq 0.
\end{align*}$$

What this notation
means is that $Q$ and $M$ are *positive semidefinite* and that $R$ is
*positive definite*
(<https://en.wikipedia.org/wiki/Positive-definite_matrix>). We will
ignore these terms for now, noting only that this is similar to
requiring (for example) that $r>0$ in order for the function $ru^{2}$ to
have a minimum.

### Solution to the problem

The "Solution approach" Section tells us to solve the LQR problem in two steps.
First, we find a function $v(t,x)$ that satisfies the HJB equation. Here
is that equation, with the functions $f$, $g$, and $h$ filled in from
the "Statement of the problem" Section:

$$
\begin{align*}
0 = \frac{\partial v(t,x)}{\partial t} + \mathop{\mathrm{minimum}}_{u} \left\{ \frac{\partial v(t,x)}{\partial x} (Ax+Bu)+x^{T}Qx+u^{T}Ru \right\}, \qquad v(t_{1},x) = x^{T}Mx.
\end{align*}
$$

What function $v$ might solve this equation? Look at the boundary
condition. At time $t_{1}$,

$$
\begin{align*}
v(t_{1},x) = x^{T}Mx.
\end{align*}$$

This function has
the form

$$
\begin{align*}
v(t,x) = x^{T}P(t)x
\end{align*}
$$

for some symmetric matrix $P(t)$ that
satisfies $P(t_{1})=M$. So let's "guess" that this form is the solution
we are looking for, and see if it satisfies the HJB equation. Before
proceeding, we need to compute the partial derivatives of $v$:

$$
\begin{align*}
\frac{\partial v}{\partial t} = x^{T} \dot{P} x \qquad\qquad \frac{\partial v}{\partial x} = 2x^{T}P
\end{align*}$$

This is matrix calculus (e.g., see
<https://en.wikipedia.org/wiki/Matrix_calculus> or Chapter A.4.1 of
<http://web.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf>). The result on
the left should surprise no one. The result on the right is the matrix
equivalent of $\partial (px^{2}) / \partial x = 2px$ (you could check
that this result is correct by considering an example). Plug these
partial derivatives into HJB and we have

$$
\begin{align*}
\tag{7}
0 &=  x^{T} \dot{P} x + \mathop{\mathrm{minimum}}_{u} \left\{ 2x^{T}P (Ax+Bu)+x^{T}Qx+u^{T}Ru \right\} \\
&= x^{T} \dot{P} x + \mathop{\mathrm{minimum}}_{u} \left\{ x^{T} ( 2 P A + \ Q ) x+2x^{T}PBu+u^{T}Ru \right\}
\end{align*}$$

To evaluate the minimum, we apply the first-derivative
test (more matrix calculus!):

$$\begin{aligned}
0 = \frac{\partial}{\partial u} \left( x^{T}(2P A + Q)x+2x^{T}PBu+u^{T}Ru \right) \\
= 2x^{T}PB+2u^{T}R \\
= 2 \left( B^{T}Px + Ru \right)^{T}.
\end{aligned}$$

This equation is
easily solved:

$$
\begin{align*}
\tag{8}
u = -R^{-1}B^{T}Px.
\end{align*}$$

Plugging this back into (7), we have

$$\begin{aligned}
0
&=  x^{T} \dot{P} x + \mathop{\mathrm{minimum}}_{u} \left\{ x^{T}(2P A + Q)x+2x^{T}PBu+u^{T}Ru \right\} \\
&= x^{T} \dot{P} x + \left( x^{T}(2P A + Q)x+2x^{T}PB(-R^{-1}B^{T}Px)+(-R^{-1}B^{T}Px)^{T}R(-R^{-1}B^{T}Px) \right) \\
&= x^{T} \dot{P} x + \left( x^{T}(2P A + Q)x-2x^{T}PBR^{-1}B^{T}Px+x^{T}PBR^{-1}B^{T}Px \right) \\
&= x^{T} \dot{P} x + x^{T}(2P A + Q)x-x^{T}PBR^{-1}B^{T}Px \\
&= x^{T} \dot{P} x + x^{T}(P A+A^{T}P + Q)x-x^{T}PBR^{-1}B^{T}Px \\[0.5em]
&\qquad\qquad \dotsc\text{because } x^{T}(N+N^{T})x=2x^{T}Nx \text{ for any } N \text{ and } x\dotsc \\[0.5em]
&= x^{T} \left( \dot{P} + P A+A^{T}P + Q - PBR^{-1}B^{T}P \right)x.\end{aligned}$$

In order for this equation to be true for any $x$, it must be the case
that

$$
\begin{align*}
\dot{P} = PBR^{-1}B^{T}P-P A-A^{T}P - Q
\end{align*}$$

In summary, we have
found that

$$
\begin{align*}
v(t,x) = x^{T}P x
\end{align*}$$

solves the HJB equation, where $P$ is
found by integrating the matrix differential equation

$$
\begin{align*}
\dot{P} = PBR^{-1}B^{T}P-P A-A^{T}P - Q
\end{align*}$$

backward in time, starting
from

$$
\begin{align*}
P(t_{1}) = M.
\end{align*}$$

Now that we know $v$, we can find $u$. Wait, we
already did that! The minimizer in the HJB equation is
(8). This choice of input has the form

$$
\begin{equation}
u = -Kx
\end{equation}$$

for

$$ \begin{aligned} K = R^{-1}B^{T}P. \end{aligned}$$

### LQR Code
Let's take a look at how to implement LQR using Python. The basic method is shown below:

```python
import numpy as np
from scipy import linalg

def lqr(A, B, Q, R):
    P = linalg.solve_continous_are(A, B, Q, R)  # Solves the continuous algebraic Riccati equation
    K = linalg.inv(R) @ B.T @ P  # Generates gain matrix
    return K
```

# State estimation

## What is a state-space model with output?

All state-space models we have seen so far have had this form:

$$\dot{x} = Ax+Bu$$

From now on, we will consider state-space models with this form instead:

$$\begin{align*} \dot{x} &= Ax + Bu \\ y &= Cx + Du \end{align*}$$

We have added one more variable (along with the state $x$ and the input $u$):

* $y$ is the **output**

Like the state and the input, the output is a function of time --- we will write $y(t)$ when we want to make the time-dependence explicit --- and may have more than one element, so is in general a vector (represented by a matrix of numbers). Suppose, in particular, that:

* the output $y$ has $n_y$ elements

Then, we would represent it as a column matrix:

$$y = \begin{bmatrix} y_1 \\ \vdots \\ y_{n_y} \end{bmatrix}$$

Like $A$ and $B$, the constants $C$ and $D$ have to be matrices:

* $C$ is a constant matrix of size $n_y \times n_x$
* $D$ is a constant matrix of size $n_y \times n_u$

The state-space model remains both **linear** and **time-invariant**, because $y$ is a linear function of $x$ and $u$ and because $C$ and $D$ are constant.

<div class="alert alert-warning">
As usual, other people may use other symbols for both the variables and constants in a state-space model. For example:

$$
\begin{align*}
\dot{z} &= Ez + Fv \\
w &= Gz + Hv
\end{align*}
$$

This is also a "state-space model," in which the state is $z$, the input is $v$, and the output is $w$.
</div>

Outputs can be used to model a variety of different things. We will use them to model sensor measurements for the purpose of state estimation.


## How do I linearize a sensor model?

You already know how to [linearize a dynamic model](#how-do-i-put-a-system-in-state-space-form):

* Rewrite the dynamic model as a set of first-order ODEs:

$$\dot{m} = f(m, n)$$

* Choose an equilibrium point $m_e, n_e$ among the solutions to this equation:

$$0 = f(m_e, n_e)$$

* Define the state and input in terms of the equilibrium point:

$$x = m - m_e \qquad\qquad u = n - n_e$$

* Compute $A$ and $B$ as the Jacobian of $f$ with respect to $m$ and $n$ respectively, evaluated at the equilibrium point:

$$
A = \frac{\partial f}{\partial m}\biggr\rvert_{(m_{e},n_{e})}
\qquad\qquad
B = \frac{\partial f}{\partial n}\biggr\rvert_{(m_{e},n_{e})}
$$

The result is a linear approximation

$$\dot{x} = Ax + Bu$$

to the nonlinear dynamic model

$$\dot{m} = f(m, n)$$

that is accurate near the equilibrium point.

We can use the same process to linearize a sensor model:

**Step 1.** Rewrite the sensor model as follows:

$$o = g(m, n).$$

In this expression, the variable $o$ is a vector of sensor measurements. It is a function of time and can have more than one element --- so, just like $m$ and $n$, it should be represented as a column matrix. Just like the function $f(m, n)$ that describes the dynamic model, the function $g(m, n)$ that describes the sensor model will often be nonlinear.

**Step 2.** Define the output as follows:

$$y = o - g(m_e, n_e).$$

Note that $y$ measures the *difference* between what the sensor measurements are and what these measurements would be if the system were at equilibrium. In particular, the output is *zero* when the system is at equilibrium even if the measurements are not.

**Step 3.** Compute $C$ and $D$ as follows:

$$
C = \frac{\partial g}{\partial m}\biggr\rvert_{(m_{e},n_{e})}
\qquad\qquad
D = \frac{\partial g}{\partial n}\biggr\rvert_{(m_{e},n_{e})}.
$$

Why does this make sense? Just like we took a first-order [Taylor's series expansion](https://en.wikipedia.org/wiki/Taylor_series) about $m_e, n_e$ to approximate the function $f(m, n)$ that describes the dynamic model, let's take a first-order series expansion about $m_e, n_e$ to approximate the function $g(m, n)$ that describes the sensor model:

$$
\begin{aligned}
y
&= o - g(m_e, n_e) \\
&= g(m, n) - g(m_e, n_e) \\
&\approx \left( g(m_e, n_e) + \frac{\partial g}{\partial m}\biggr\rvert_{(m_{e},n_{e})} \left( m - m_e \right) + \frac{\partial g}{\partial n}\biggr\rvert_{(m_{e},n_{e})} \left( n - n_e \right) \right) - g(m_e, n_e) \\
&= \frac{\partial g}{\partial m}\biggr\rvert_{(m_{e},n_{e})} \left( m - m_e \right) + \frac{\partial g}{\partial n}\biggr\rvert_{(m_{e},n_{e})} \left( n - n_e \right) \\
&= C x + D u
\end{aligned}
$$

So, with this choice of $C$ and $D$, we have produced a linear approximation

$$y = Cx + Du$$

to the nonlinear sensor model

$$o = g(m, n)$$

that is accurate near the equilibrium point.

#### Example (linear sensor model)

Consider again a system with the following dynamic model:

$$\ddot{q} + 3\sin q = \tau.$$

[We already showed](#example-second-order) how to rewrite this dynamic model as

$$
\begin{bmatrix} \dot{q} \\ \dot{v} \end{bmatrix} = \begin{bmatrix} v \\ -3\sin q + \tau \end{bmatrix}
$$

and how to linearize it about the equilibrium point

$$
q_e = \pi / 2 \qquad v_e = 0 \qquad \tau_e = 3
$$

to produce the state-space model

$$
\dot{x} = Ax+Bu
$$

where

$$
A = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix}
\qquad\qquad
B = \begin{bmatrix} 0 \\ 1 \end{bmatrix}
$$

and

$$
x = \begin{bmatrix} q - q_e \\ v - v_e \end{bmatrix} \qquad\qquad u = \begin{bmatrix}\tau - \tau_e\end{bmatrix}.
$$

Now suppose we have sensors that allow the measurement of

$$q.$$

Let's apply our method to put this measurement in state-space form.

First, we rewrite the measurement in the form $o = g(m, n)$, i.e., as a vector-valued function of $m$ and $n$:

$$
o
=
\begin{bmatrix} q \end{bmatrix}.
$$



Then, we define the output as the difference between the value of this function and what the value would be at equilibrium:

$$
\begin{aligned}
y
&= o - g\left( \begin{bmatrix} q_e \\ v_e \end{bmatrix}, \begin{bmatrix} \tau_e \end{bmatrix} \right) \\
&= \begin{bmatrix} q \end{bmatrix} - \begin{bmatrix} q_e \end{bmatrix} \\
&= \begin{bmatrix} q - q_e \end{bmatrix}.
\end{aligned}
$$

Finally, we compute $C$ and $D$ by taking Jacobians:

$$
\begin{aligned}
C
&= \frac{\partial g}{\partial \begin{bmatrix} q \\ v \end{bmatrix}}\biggr\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \left.\begin{bmatrix} \dfrac{\partial(q)}{\partial q} & \dfrac{\partial(q)}{\partial v} \end{bmatrix}\right\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \left.\begin{bmatrix} 1 & 0 \end{bmatrix}\right\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \begin{bmatrix} 1 & 0 \end{bmatrix} \\[1em]
D
&= \frac{\partial g}{\partial \begin{bmatrix} \tau \end{bmatrix}}\biggr\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \left.\begin{bmatrix} \dfrac{\partial(q)}{\partial \tau} \end{bmatrix}\right\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \left.\begin{bmatrix} 0 \end{bmatrix}\right\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \begin{bmatrix} 0 \end{bmatrix}
\end{aligned}
$$

The resulting state-space model is

$$
\begin{aligned}
y &= Cx + Du \\
&= \begin{bmatrix} 1 & 0 \end{bmatrix}x + \begin{bmatrix} 0 \end{bmatrix}u.
\end{aligned}
$$

Note that the original sensor model was linear, so there was no approximation here. We could probably have skipped the entire process of linearization and written the system in state-space form bny inspection. However, just as was true when putting dynamic models in state-space form (see [example](#example-first-order)), it is nice to know that "linearization" still works even in this simple case.

#### Example (nonlinear sensor model)

Consider a system with the [same dynamic model as before](#example-linear-sensor-model), but now suppose we have sensors that allow the measurement of

$$\dfrac{\cos q}{\sin q}.$$

Again, let's apply our method to put this measurement in state-space form.

First, we rewrite the measurement in the form $o = g(m, n)$, i.e., as a vector-valued function of $m$ and $n$:

$$
o
=
\begin{bmatrix} \cos q / \sin q \end{bmatrix}.
$$

Then, we define the output as the difference between the value of this function and what the value would be at equilibrium:

$$
\begin{aligned}
y
&= o - g\left( \begin{bmatrix} q_e \\ v_e \end{bmatrix}, \begin{bmatrix} \tau_e \end{bmatrix} \right) \\
&= \begin{bmatrix} \cos q / \sin q \end{bmatrix} - \begin{bmatrix} \cos q_e / \sin q_e \end{bmatrix} \\
&= \begin{bmatrix} (\cos q / \sin q) - (\cos q_e / \sin q_e) \end{bmatrix}.
\end{aligned}
$$

Finally, we compute $C$ and $D$ by taking Jacobians:

$$
\begin{aligned}
C
&= \frac{\partial g}{\partial \begin{bmatrix} q \\ v \end{bmatrix}}\biggr\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \left.\begin{bmatrix} \dfrac{\partial(\cos q / \sin q)}{\partial q} & \dfrac{\partial(\cos q / \sin q)}{\partial v} \end{bmatrix}\right\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \left.\begin{bmatrix} -1 / (\sin q)^2 & 0 \end{bmatrix}\right\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \begin{bmatrix} -1 & 0 \end{bmatrix} \\[1em]
D
&= \frac{\partial g}{\partial \begin{bmatrix} \tau \end{bmatrix}}\biggr\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \left.\begin{bmatrix} \dfrac{\partial(\cos q / \sin q)}{\partial \tau} \end{bmatrix}\right\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \left.\begin{bmatrix} 0 \end{bmatrix}\right\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \begin{bmatrix} 0 \end{bmatrix}
\end{aligned}
$$

The resulting state-space model is

$$
\begin{aligned}
y &= Cx + Du \\
&= \begin{bmatrix} -1 & 0 \end{bmatrix}x + \begin{bmatrix} 0 \end{bmatrix}u.
\end{aligned}
$$

<div class="alert alert-warning">
In both of the previous two examples, we found that $D = 0$. This is so often the case that we may just write

$$y = Cx$$

instead of

$$y = Cx + Du$$

when describing a sensor model in state-space form.
</div>


# Optimal Observer Derivation (a.k.a the Kalman Filter, a very famous thing)

These notes were originally written by T. Bretl and were transcribed for this reference page by S. Bout.

## Statement of the problem {#secStatement}

Here is the deterministic, finite-horizon, continuous-time *Kalman
Filter (KF)* problem---i.e., the optimal control problem that one would
solve to produce an optimal observer:

$$\begin{aligned}
\mathop{\mathrm{minimize}}_{x(t_{1}),n_{[t_{0},t_{1}]},d_{[t_{0},t_{1}]}}
&\qquad
n(t_{0})^{T}M_{o}n(t_{0})+
\int_{t_{0}}^{t_{1}} \left( n(t)^{T}Q_{o}n(t)+d(t)^{T}R_{o}d(t) \right) dt \\
\text{subject to}
&\qquad
\dot{x}(t) = Ax(t)+Bu(t)+d(t) \\
&\qquad
y(t) = Cx(t)+n(t)\end{aligned}$$

The interpretation of this problem is
as follows. The current time is $t_{1}$. You have taken measurements
$y(t)$ over the time interval $[t_{0}, t_{1}]$. You are looking for
noise $n(t)$ and disturbance $d(t)$ over this same time interval and for
an estimate $x(t_{1})$ of the current state that would best explain
these measurements.

The matrices $Q_{o}$, $R_{o}$, and $M_{o}$ are parameters that can be
used to trade off noise (the difference between the measurements and
what you expect them to be) with disturbance (the difference between the
time derivative of the state and what you expect it to be). These
matrices have to be symmetric, have to be the right size, and also have
to satisfy the following conditions in order for the KF problem to have
a solution:

$$Q_{o} \geq 0 \qquad\qquad R_{o}>0 \qquad\qquad M_{o}\geq 0.$$

Just as
with the LQR problem, this notation means is that $Q_{o}$ and $M_{o}$
are *positive semidefinite* and that $R_{o}$ is *positive definite*
([see
wikipedia](https://en.wikipedia.org/wiki/Positive-definite_matrix)).

By plugging in the expression for $n(t)$ that appears in the constraint,
this optimal control problem can be rewritten as

$$\begin{aligned}
\mathop{\mathrm{minimize}}_{x(t_{1}),d_{[t_{0},t_{1}]}}
&\qquad
(Cx(t_{0}) - y(t_{0}))^{T}M_{o}(Cx(t_{0}) - y(t_{0}))\\
&\qquad\qquad
+\int_{t_{0}}^{t_{1}} \left( (Cx(t) - y(t))^{T}Q_{o}(Cx(t) - y(t))+d(t)^{T}R_{o}d(t) \right) dt \\
\text{subject to}
&\qquad
\dot{x}(t) = Ax(t)+Bu(t)+d(t)\end{aligned}$$

It is an optimal control
problem, just like LQR---if you define

$$\begin{aligned}
f(t,x,d) &= Ax+Bu(t)+d \\
g(t,x,d) &= (Cx-y(t))^{T}Q_{o}(Cx-y(t))+d^{T}R_{o}d(t) \\
h(t,x) &= (Cx-y(t))^{T}M_{o}(Cx-y(t))\end{aligned}$$

then you see that
this problem has the general form

$$\begin{aligned}
\underset{x(t_1),d_{[t_0,t_1]}}{\text{minimize}}
&\qquad
h(t_0,x(t_{0}))+
\int_{t_{0}}^{t_{1}} g(t,x(t),d(t)) dt \\
\text{subject to}
&\qquad
\frac{dx(t)}{dt}=f(t,x(t),d(t)).\end{aligned}$$

There are four
differences between this form and the one we saw when solving the LQR
problem:

-   The "input" in this problem is $u$, not $d$.

-   The "current time" is $t_{1}$ and not $t_{0}$.

-   The final state---i.e., the state at the current time---is *not*
    given. Indeed, the point here is to *choose* a final state
    $x(t_{1})$ that best explains $u(t)$ and $y(t)$.

-   The functions $f$, $g$, and $h$ vary with time (because they have
    parameters in them---$u(t)$ and $y(t)$---that are functions of
    time).

Because of these four differences, the HJB equation for a problem of
this form is

$$0 = -\frac{\partial v(t,x)}{\partial t} + \mathop{\mathrm{minimum}}_{d} \left\{ -\frac{\partial v(t,x)}{\partial x} f(t,x,d)+g(t,x,d) \right\}, \qquad v(t_{0},x) = h(t_{0}, x(t_{0})).$$

Note the change in sign of both the first term outside the minimum and
the first term inside the minimum---this is because we are effectively
solving an optimal control problem in which time flows backward (from
the current time $t_{1}$ to the initial time $t_{0}$, instead of from
the current time $t_{0}$ to the final time $t_{1}$). It is possible to
derive this form of the HJB equation in exactly the same way as it was
done in the notes on LQR.

## Solution to the problem

As usual, our first step is to find a function $v(t, x)$ that satisfies
the HJB equation. Here is that equation, with the functions $f$, $g$,
and $h$ filled in:

$$\begin{aligned}
0 &= -\frac{\partial v(t,x)}{\partial t} + \mathop{\mathrm{minimum}}_{d} \biggl\{ -\frac{\partial v(t,x)}{\partial x} \left(Ax+Bu(t)+d\right) \\
&\qquad\qquad\qquad\qquad\qquad\qquad\qquad +(Cx-y(t))^{T}Q_{o}(Cx-y(t))+d(t)^{T}R_{o}d(t) \biggr\} \\
v(t_{0},x) &= (Cx(t_{0})-y(t_{0}))^{T}M_{o}(Cx(t_{0})-y(t_{0})).
\end{aligned}$$

Expand the boundary condition:

$$\begin{aligned}
v(t_{0},x)
&= (Cx(t_{0})-y(t_{0}))^{T}M_{o}(Cx(t_{0})-y(t_{0})) \\
&= x(t_{0})^{T} C^{T}M_{o}C x(t_{0}) - 2 y(t_{0})^{T}M_{o}C^{T}x(t_{0}) + y(t_{0})^{T}M_{o}y(t_{0})
\end{aligned}$$

This function has the form

$$v(t, x) = x^{T}P(t)x +2 o(t)^{T} x + w(t)$$

for some symmetric matrix $P(t)$ and some other matrices $o(t)$ and
$w(t)$ that satisfy the following boundary conditions:

$$P(t_{0}) = C^{T}M_{o}C
\qquad
o(t_{0}) = -CM_{o}y(t_{0})
\qquad
w(t_{0}) = y(t_{0})^{T}M_{o}y(t_{0}).$$

Let's "guess" that this form of
$v$ is the solution we are looking for, and see if it satisfies the HJB
equation. Before proceeding, we need to compute the partial derivatives
of $v$:

$$\frac{\partial v}{\partial t} = x^{T} \dot{P} x + 2 \dot{o}^{T} x + \dot{w} \qquad\qquad \frac{\partial v}{\partial x} = 2x^{T}P + 2o^{T}$$

Here again---just as for LQR---we are applying [matrix
calculus](https://en.wikipedia.org/wiki/Matrix_calculus). Plug these
partial derivatives into HJB and we have

$$\begin{align*}
0 &=  -\left(x^{T} \dot{P} x + 2 \dot{o}^{T} x + \dot{w}\right) + \mathop{\mathrm{minimum}}_{d} \biggl\{ -\left(2x^{T}P+ 2o^{T}\right) (Ax+Bu+d) \\
&\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad +(Cx-y)^{T}Q_{o}(Cx-y)+d^{T}R_{o}d \biggr\} \\
\tag{1}
&= -\left(x^{T} \dot{P} x + 2 \dot{o}^{T} x + \dot{w}\right) + \mathop{\mathrm{minimum}}_{d} \biggl\{ d^{T}R_{o}d - 2(Px + o)^{T}d \\
&\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad -2 (x^{T} P+o^{T}) (Ax+Bu) \\
&\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad + (Cx-y)^{T}Q_{o}(Cx-y) \biggr\}
\end{align*}$$

To evaluate the minimum, we apply the first-derivative
test (more matrix calculus!):

$$\begin{aligned}
0
&= \frac{\partial}{\partial d} \left( d^{T}R_{o}d - 2(Px + o)^{T}d -2 (x^{T} P+o^{T}) (Ax+Bu) + (Cx-y)^{T}Q_{o}(Cx-y) \right) \\
&= 2d^{T}R_{o}-2(Px+o)^{T}.
\end{aligned}$$

This equation is easily
solved:

$$\begin{align*}
\tag{2}
d = R^{-1}(Px+o).
\end{align*}$$

Plugging this back into (1), we have

$$\begin{aligned}
0
&=  -\left(x^{T} \dot{P} x + 2 \dot{o}^{T} x + \dot{w}\right) + \mathop{\mathrm{minimum}}_{d} \biggl\{ -\left(2x^{T}P+ 2o^{T}\right) (Ax+Bu+d) \\
&\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad +(Cx-y)^{T}Q_{o}(Cx-y)+d^{T}R_{o}d \biggr\} \\
&= -(x^{T} \dot{P} x + 2 \dot{o}^{T} x + \dot{w}) - (Px+o)^{T}R_{o}^{-1}(Px+o) \\
&\qquad\qquad -2 (x^{T} P+o^{T}) (Ax+Bu) + (Cx-y)^{T}Q_{o}(Cx-y) \\
&= x^{T}\left( -\dot{P} - PR_{o}^{-1}P -2PA + C^{T}Q_{o}C \right)x \\
&\qquad\qquad + 2x^{T} \left( -\dot{o} - PR_{o}^{-1}o - PBu -C^{T}Q_{o}y - A^{T}o \right) \\
&\qquad\qquad\qquad +\left( -\dot{w} - o^{T}R_{o}^{-1}o -2o^{T}Bu + y^{T}Q_{o}y \right) \\
&= x^{T}\left( -\dot{P} - PR_{o}^{-1}P -PA - A^{T}P + C^{T}Q_{o}C \right)x \\
&\qquad\qquad + 2x^{T} \left( -\dot{o} - PR_{o}^{-1}o - PBu -C^{T}Q_{o}y - A^{T}o \right) \\
&\qquad\qquad\qquad +\left( -\dot{w} - o^{T}R_{o}^{-1}o -2o^{T}Bu + y^{T}Q_{o}y \right)
\end{aligned}$$

where the last step is because

$$x^{T}(N+N^{T})x=2x^{T}Nx \text{ for any } N \text{ and } x.$$

In order
for this equation to be true for any $x$, it must be the case that

$$\begin{aligned}
\dot{P} &= -PR_{o}^{-1}P-P A-A^{T}P +C^{T}Q_{o}C \\
\dot{o} &= - PR_{o}^{-1}o - PBu -C^{T}Q_{o}y - A^{T}o \\
\dot{w} &= - o^{T}R_{o}^{-1}o -2o^{T}Bu + y^{T}Q_{o}y.
\end{aligned}$$

In summary, we have found that

$$v(t, x) = x^{T}P(t)x +2 o(t)^{T} x + w(t)$$

solves the HJB equation,
where $P$, $o$, and $w$ are found by integrating the above ODEs forward
in time, starting from

$$P(t_{0}) = C^{T}M_{o}C
\qquad
o(t_{0}) = -CM_{o}y(t_{0})
\qquad
w(t_{0}) = y(t_{0})^{T}M_{o}y(t_{0}).$$

The optimal choice of state
estimate at time $t$ is the choice of $x$ that minimizes $v(t, x)$, that
is, the solution to

$$\mathop{\mathrm{minimize}}_{x} \qquad x^{T}P(t)x +2 o(t)^{T} x + w(t).$$

We can find the solution to this problem by application of the first
derivative test, with some matrix calculus:

$$\begin{aligned}
0
&= \frac{\partial}{\partial x} \left( x^{T}Px +2 o^{T} x + w \right) \\
&= 2x^{T}P + 2o^{T},
\end{aligned}$$

 implying that

 $$x = -P^{-1}o.$$

Let's call this solution $\widehat{x}$. Note that we can, equivalently,
write

$$0 = P\widehat{x} + o.$$

Suppose we take the time derivative of
this expression, plugging in what we found earlier for $\dot{P}$ and
$\dot{o},$ as well as plugging in yet another version of this same
expression, $o = -P\widehat{x}$:

$$\begin{aligned}
0
&= \dot{P} \widehat{x} + P\dot{\widehat{x}} + \dot{o} \\
&= \left( -PR_{o}^{-1}P-P A-A^{T}P +C^{T}Q_{o}C \right)\widehat{x} + P\dot{\widehat{x}} - PR_{o}^{-1}o - PBu -C^{T}Q_{o}y - A^{T}o \\
&= -PR_{o}^{-1}P\widehat{x}-P A\widehat{x}-A^{T}P\widehat{x} +C^{T}Q_{o}C \widehat{x} + P\dot{\widehat{x}} + PR_{o}^{-1}P\widehat{x} - PBu -C^{T}Q_{o}y + A^{T}P\widehat{x} \\
&= P\dot{\widehat{x}} -PA\widehat{x}-PBu + C^{T}Q_{o}(C\widehat{x} - y) \\
&= P \left( \dot{\widehat{x}} - A\widehat{x} - Bu + P^{-1}C^{T}Q_{o}(C\widehat{x} - y) \right).
\end{aligned}$$

For this equation to hold for any $P$, we must have

$$\dot{\widehat{x}} = A\widehat{x} + Bu - P^{-1}C^{T}Q_{o}(C\widehat{x} - y).$$

**Behold!** This is our expression for an optimal observer, if we define

$$L = P^{-1}C^{T}Q_{o}.$$

Finally, suppose we take the limit as
$t_{0}\rightarrow-\infty$, so assume an infinite horizon. It is a fact
that $P$ tends to a steady-state value, and so $L$ does as well. When
this happens, $\dot{P} = 0$, and so the steady-state value of $P$ is the
solution to the *algebraic* equation

$$0 = -PR_{o}^{-1}P-P A-A^{T}P +C^{T}Q_{o}C.$$

It is customary to write
these last two equations in a slightly different way. In particular,
suppose we pre- and post-multiply both sides of this last equation by
$P^{-1}$, and define

$$P_{o} = P^{-1}$$

Then, we have

$$L = P_{o}C^{T}Q_{o}$$

and

$$0 = P_{o}C^{T}Q_{o}CP_{o}-AP_{o}-P_{o}A^{T} -R_{o}^{-1}.$$

## Summary

An optimal observer---a deterministic, infinite-horizon, continuous-time
Kalman Filter---is given by

$$\dot{\widehat{x}} = A\widehat{x} + Bu - L(C\widehat{x} - y).$$

where

$$L = P_{o}C^{T}Q_{o}$$

and $P_{o}$ satisfies

$$0 = P_{o}C^{T}Q_{o}CP_{o}-AP_{o}-P_{o}A^{T} -R_{o}^{-1}.$$

## Comparison between LQR and KF (i.e., why you can use "LQR" in Python to compute an optimal observer)

An optimal controller is given by

$$u = -Kx$$

where

$$\begin{align*}
\tag{3}
K = R_{c}^{-1}B^{T}P_{c}
\end{align*}$$

and $P_{c}$ satisfies

$$\begin{align*}
\tag{4}
0 = P_{c}BR_{c}^{-1}B^{T}P_{c} - P_{c}A - A^{T}P_{c}-Q_{c}.
\end{align*}$$

An optimal observer is given by

$$\dot{\widehat{x}} = A\widehat{x} + Bu - L(C\widehat{x} - y).$$

where

$$\tag{5}
L = P_{o}C^{T}Q_{o}
$$

and $P_{o}$ satisfies

$$\tag{6}
0 = P_{o}C^{T}Q_{o}CP_{o}-AP_{o}-P_{o}A^{T} -R_{o}^{-1}.
$$

Take the transpose of (5) and---remembering that $P_{o}$ and $Q_{o}$ are symmetric---we get

$$\tag{7}
L^{T} = Q_{o}CP_{o}.
$$

Take the transpose of (6)
and---remembering that $R_{o}$ is also symmetric---we get

$$\tag{8}
0 = P_{o}C^{T}Q_{o}CP_{o}-P_{o}A^{T}-AP_{o} -R_{o}^{-1}.
$$

Compare (3) and (4) with (7) and (8). They are **exactly the same** if we make the
following replacements:

-   replace $K$ with $L^{T}$

-   replace $A$ with $A^{T}$

-   replace $B$ with $C^{T}$

-   replace $Q_{c}$ with $R_{o}^{-1}$

-   replace $R_{c}$ with $Q_{o}^{-1}$

**This** is the reason why

```python
L = lqr(A.T, C.T, linalg.inv(Ro), linalg.inv(Qo)).T
```

produces an optimal observer, just like

```python
K = lqr(A, B, Qc, Rc)
```

produces an optimal controller. **WWWWWOOOOOWWWWW!!!!!!!**

Remember that:
```python
import numpy as np
from scipy import linalg

def lqr(A,B,Q,R):
    P=linalg.solve_continuous_are(A,B,Q,R)
    K=linalg.inv(R) @ B.T @ P
    return K
```



# Python Tutorials
T. Bretl, S. Bout, J. Virklan
## Lists and Numpy Arrays

A basic object in python is a list. Lists are used for storing data
related to a variable. An example of a list is:

``` {language="Python"}
x = [3, 1, -1, 10]
```

The indices of a list are stored like an array in other languages. To
access the values in the list, one would type the variable with the list
index in square brackets. Python indexes start at zero, so the first
element of a list is the 0th index, the second is the 1st index and so
on. To access the 1 stored in the second index of the list above, one
would write x\[1\], and this would return the value of 1. Lists in
python require no packages to utilize as they are a base feature of
python.

Not all indices of a list need be number values, the lists can store any
number of data types, for instance, a string. Lists can also store
values in 2-dimensions, much like a 2d array. In python, these are
called list of lists and an example of one is shown below.

``` python
x = [[1, 2], [3, 4]]
```

In the example above, the list of lists, x, has 2 elements. The first
element is the list \[1, 2\] and the second element is \[3, 4\]. To
print out the variable x, one would use the print() function. an example
of printing x is shown below.

``` python
print(x)
```

Python has various packages that can be imported and utilized. One key
package used in scientific computing is NumPy. An example of bringing
NumPy to utilize in code is shown below.

``` python
import numpy as np
```

This brings numpy into the code it is being utilized in. As np allows
the user to call numpy's libraries using np as a shortened phrase. To
call a function or variable from numpy, one would use np.FUNCTION. the
np tells Python where the function is located to utilize it. An example
is shown below.

``` python
pi = np.pi
```

This brings a value for pi that is stored in the numpy packages as
python does not automatically store pi in a variable. The value for pi
can now be used as pi throughout the code, and printing this value would
show pi's value of 3.14\...

A more valuable use of numpy comes from the numpy arrays, and is what
makes numpy so valuable. An example of a list of lists being used to
create a numpy array is shown below.

``` python
import numpy as np

x = [[1, 2], [3, 4]]
A = np.array(x)
```

Numpy need only be imported once in a document or file utilizing python
code. the list of lists, x, from above is now stored in a numpy array
and can be utilized with numpy's matrix operations (these will be
discussed below). All numbers in numpy arrays must have the same data
type, and are typically stored as float64 if a decimal point is used in
their values, giving a high number of precision. If no decimal point is
used in the values, they are stored as an integer by default. In the
example above, the element \[1, 2\] is the first row of the array.
Subsequent rows must be separated by a comma before defining the next
row. While numpy arrays utilize a list of lists to create an array, they
are two separate objects, and this should be kept in mind when using
list of lists to create a numpy array.

## The Sympy Package

Another useful package from python utilized in science and engineering
is the SymPy package. This package allows creation of symbolic variables
that can be used for things such as linearization and solving equations.
Symbolic variables are stored in their own type of matrix from sympy's
libraries called a symbolic matrix. An example of creating symbolic
variables and storing them in a symbolic matrix is shown below:

``` python
import sympy as sym
import numpy as np

q, v, tau = sym.symbols('q, v, tau')

f = sym.Matrix([v, -3*sym.sin(q) + tau]
```

Note that q, v, and tau are defined using comma separated variable
definitions, and the sym.symbols package takes each stored in a string
separated by commas to create the symbolics. This principle can be
extended to however many variables are necessary.

To linearize the matrix, equilibrium values must be defined to input
into the linearized matrix and create a matrix of purely numbers. Next,
sympy's lamdify function is used to create a function from the symbolics
that values can be input in. Finally, this function can substitute the
equilibrium values and return a equilibrium points from the function.

``` python
q_e = 0.5 * np.pi
v_e = 0.
tau_e = 3.

f_num = sym.lamdify([q, v, tau], f)

f_num(q_e, v_e, tau_e)
```

here, sym.lamdify takes the arguments of symbolic variables utilized in
f as a list and the function of those symbolic variables f. Lamdify then
creates a function that can be used to plug in choices of equilibrium
values.

We can use this, and the jacobian function from sympy to create
linearized matrices.

``` python
f.jacobian([q,v])
```

creates a matrix of partial derivatives with respect to q and v from f.

This can be extended to a state space model for the A matrix as:

``` python
# create lambda function
A_num = sym.lamdify([q, v, tau], f.jacobian([q,v]))

# evaluate lambda function at the specified equilibrium points
A = A_num(q_e, v_e, tau_e)

print(A)

np.set_printoptions(suppress=True)
print(A)
```

The latter portion of the code suppresses scientific notation to make
the matrix more readable when printing. We have now created an A matrix
for the state-space model. The B matrix is, likewise, created as:

``` python
B_num = sym.lamdify([q, v, tau], f.jacobian([tau]), 'numpy')

B = B_num(q_e, v_e, tau_e)

print(B)
```

The values printed as B are not floats in this case, so they must be
converted before using.

``` python
A = A.astype(float)
B = B.astype(float)

print(A)
print(B)
```

For the purposes of entering data in prarielearn, it is desired to print
the numpy arrays to python lists. This can be done easily.

``` python
print(A.tolist())
print(B.tolist())
```

## Matrix Operations

Matrices in numpy can be added and subtracted given they have the same
dimensions. Be warned, matrices that aren't the same size can still be
added and subtracted in Python without an error being thrown. It is
important to be careful as to what matrices are being utilized in
computation. an example of adding and subtracting matrices is shown
below, along with an example of adding non-compatible sized matrices.

``` python
import numpy as np

L = np.array([[0., 1.], [-3., 2.]])
M = np.array([[2., 0.], [-1., 1.]])
N = np.array([[-1.], [4.]])

# compatible sizes
print(L+M)
print(L-M)

print(M+N) # non-compatible size
```

The non-compatible size calculation, in this case, adds N to each column
of N. This is because the shape of N is broadcast to the shape of M.
This speeds up computation, but can introduce bugs in python code if
care is not taken.

Another three operations that can be performed in numpy are transpose,
matrix multiplication and element-wise multiplication. Examples of each
are shown below.

``` python
# Transpose of M
Mtranspose = M.T
print(M)
print(Mtranspose)

# matrix multiplication of M and N
print(M @ N)

# Elementwise multiplication of M and N
print(M * N)
```

It is important to not perform element-wise multiplication when matrix
multiplication is necessary. The @ operator in numpy performs matrix
multiplication while the \* operator will perform element-wise
multiplication. Another way of performing matrix multiplication in numpy
is the matmul() function. The matrix multiplication of M with N in this
case would be np.matmul(M, N).

Multiplying a matrix by a constant value is simply the matrix times the
value using the \* operator. This will broadcast the single dimensional
constant to the entire shape of the matrix.

### Matrix vector operations

In Python, one can define a vector using a single list, or a list of
lists. It is better, in most cases, to define the vector using a single
list. This will reduce potential bugs in the code in terms of matrix
sizes when multiplying the matrices by vectors. An example of how a
vector should be defined is done below with a matrix that will be
multiplied by the vector.

``` python
# Define the 1d vector
v = np.array([2., -3.])

# Define a matrix
L = np.array([0., 1.],[-3., 2.])

print(L @ v)
```

this will print an array as \[\[-3.\],\[-12\]\], the matrix
multiplication of L and v. The matrix multiplication operator will not
work with matrices and/or vectors with incompatible sizes, for instance,
the matrix N from above multiplied with v. We can, however, perform this
operation:

``` python
print(N.T @ v)
```

because transposing the matrix N makes the size compatible with the
vector v.

### Matrix Exponential

suppose we want to find $e^M$, the exponentiation of a matrix M. This
can be done using the scipy's linalg module. If numpy's exp() function
is used, the returned matrix will not be the matrix exponential.

``` python
from scipy import linalg

# the matrix exponential
expM = linalg.expm(M)
print(expM)

# not the matrix exponential
print(np.exp(M)) 
```

Use numpy's exponential for scalar exponentiation and scipy's
exponential for matrix exponentiation.

### Application to a linear system (numerical)

We desire to solve: 

$$\begin{aligned}
    \dot{x} = Fx
\end{aligned}$$

with an initial condition $x(0) = x_0$. The solution to this linear
system is:

$$\begin{aligned}
    x(t) = e^{Ft}x_0
\end{aligned}$$

This can be evaluated numerically with given values using numpy and
scipy:

``` python
F = np.array([[0., 1.],[4., 3.]])
t = 0.5
x0 = np.array([-1., 2.])

x = linalg.expM(F*t)@x0
print(x)
```

### Matrix operations using sympy

The same solution can be found using sympy:

``` python
L = sym.Matrix(L)
M = sym.Matrix(M)
N = sym.Matrix(N)
```

The stored values have floating-point numbers and it is best to be
working with rational numbers in sympy. Luckily, these can be easily
converted as shown below.

``` python
L = sym.nsimplify(L, rational=True)
M = sym.nsimplify(M, rational=True)
N = sym.nsimplify(N, rational=True)
```

The same rules for adding and subtracting matrices in numpy also apply
in sympy. Matrix multiplication can be performed using the @ operator.
However, matrix multiplication in sympy can also be performed using the
\* operator. It is STRONGLY encouraged that the @ operator is used to
stay consistent when writing code. Sympy also has a matrix exponential
operator, that is:

``` python
# perform matrix exponentiation in sympy
expM = sym.exp(M)
print(expM)

# it is helpful to simplify the answer
expM = sym.simplify(expM)
print(expM)

# convert to numpy array to compare to prior result
expMnp = np.array(expM, dtype=np.float64)
print(expMnp)
```

If we desire to create the matrix exponential of M multiplied by some
time t, we can create a symbolic for t and exponentiate that.

``` python
t = sym.symbols('t', real=True)
Mt = sym.exp(M * t)

# simplify the exponential
print(sym.simplify(Mt))

# convert simplified exponential to a list
print(Mt.tolist())
```

By establishing the symbol t as real=True, we tell sympy that we don't
want imaginary values to be taken in to the simplify function. This
doesn't necessarily work in all cases, but can be remedied using the
code provided below.

``` python
# sym.I is the imaginary unit in sympy
complex = sym.exp((-1 + 2*sym.I) * t) + sym.exp((-1 -2*sym.I) * t)

# simplifying does not get rid of complex exponentials:
print(sym.simplify(complex))

# utilizing expand_complex, the expression is simplified using Euler's formula
print(sym.expand_complex(complex))
```

This would not work at all if we hadn't told sympy to assume that $t$
was real-valued.

## Eigenvalues

Start by importing numpy (for matrix manipulation) and scipy.linalg
(for computing eigenvalues).

``` python
import numpy as np
from scipy import linalg
```

Define two square matrices to use as examples: 

$$\begin{aligned}
    F = 
    \begin{bmatrix}
    -2 & 0 \\
    3 & -1
    \end{bmatrix}
    \qquad
    G =
    \begin{bmatrix}
    3/2 & 17/2 \\
    -5/2 & -15/2
    \end{bmatrix}\end{aligned}$$

``` python
F = np.array([[-2.0, 0.0], [3.0, -1.0]])
G = np.array([[1.5, 8.5], [-2.5, -7.5]])
```

An eigenvalue of a square matrix $M$ is a complex number $s$ for which
there exists a nonzero column matrix $v≠0$ such that

$$\begin{aligned}
    Mv=vs\end{aligned}$$

Any such $v$ is called an eigenvector of $M$ that is associated with the
eigenvalue $s$. We will ignore eigenvectors for now, and focus only on
the eigenvalues.

If $M$ has size n x n , then $M$ has n eigenvalues (some of which may
be the same).

Here is how to find the two eigenvalues of the matrix $F$ for example:

``` python
s = linalg.eigvals(F)
```

The eigenvalues are returned in a 1D numpy array:

``` python
print(s)
```

The elements of this array - the eigenvalues - are complex numbers:

``` python
print(s.dtype)
```

Note that $1j$ is the imaginary unit in Python:

``` python
1j
```

To create the complex number $3−5j$, for example, we could write either
of the following things:

``` python
print(3. - 5.j)
print(3. - (5. * 1j))
```

Note that $j$ alone is not defined - you have to write $1j$:

``` python
j
```

We can get the real part of each eigenvalue:

``` python
print(s.real)
```

We can also get the imaginary part of each eigenvalue:

``` python
print(s.imag)
```

In this particular case, both eigenvalues have zero imaginary part. They
are still represented as complex numbers, however.

We can check if each eigenvalue has negative real part:

``` python
s.real < 0
```

Note the result is another array of the same length as s, with element i
containing True if s.real\[i\] \< 0 and False otherwise.

We can check if all eigenvalues have negative real part:

$$\begin{aligned}
    (s.real < 0).all()
\end{aligned}$$ 
    
This is very useful when checking
if a linear system is stable, for instance.

What about the eigenvalues of $G$? Again, this is a 2×2 matrix, so there
are two eigenvalues:

``` python
s = linalg.eigvals(G)
print(s)
```

In this case, the eigenvalues are really, truly complex, having non-zero
imaginary part. Note that the two eigenvalues are complex conjugates -
that is to say, one has the same real part and negative the imaginary
part of the other.

Again, we can easily check if all eigenvalues have negative real part:

``` python
(s.real < 0).all()
```

Why find these eigenvalues? Recall that the closed-loop system is
asymptotically stable (i.e., \"the controller works\") if and only if
all eigenvalues of $F$ have negative real part. Do you remember how to
check this condition? (If you don't, then read through this tutorial one
more time.) Is the closed-loop system stable?

## Eigenvectors

Start by importing numpy (for matrix manipulation) and scipy.linalg
(for computing eigenvalues).

``` python
import numpy as np
from scipy import linalg

# Suppress the use of scientific notation when printing small numbers
np.set_printoptions(suppress=True)
```

Define a square matrix to use as an example: 

$$\begin{aligned}
    F = 
    \begin{bmatrix}
    -2 & 0\\
    3 & -1
    \end{bmatrix}
\end{aligned}$$

``` python
F = np.array([[-2.0, 0.0], [3.0, -1.0]])
```

An eigenvalue of a square matrix $M$ is a complex number $s$ for which
there exists a nonzero column matrix $v≠0$ such that

$$\begin{aligned}
    Mv=vs\end{aligned}$$

Any such $v$ is called an eigenvector of $M$ that is associated with the
eigenvalue $s$. We will ignore eigenvectors for now, and focus only on
the eigenvalues.

If $M$ has size $n×n$ , then $M$ has $n$ eigenvalues. If all $n$
eigenvalues are distinct (so, no two eigenvalues are the same) then all
$n$ eigenvectors will be linearly independent (so, no two eigenvalues
$i$ and $j$ satisfy $v_i=kv_j$ for some non-zero real number $k∈ℝ$ ).

You have already seen how to find the eigenvalues of a square matrix:

``` python
s = linalg.eigvals(F)
print(s)
```

Here is how to find the eigenvectors of this same matrix:

``` python
s, V = linalg.eig(F)
```

The eigenvectors are returned as the columns of a 2D numpy array:

``` python
print(V)
```

The eigenvalues are also returned, in a 1D numpy array just as if we had
used linalg.eigvals:

    print(s)

Suppose we wanted to look at just the first eigenvalue. Python indices
start at zero, so the \"first eigenvalue\" is at index 0 in s:

``` python
s1 = s[0]
print(s1)
```

Similarly, the second eigenvalue is at index 1:

``` python
s2 = s[1]
print(s2)
```

Suppose we wanted to look at just the first eigenvector. This is in the
first column of $V$ . Let's sneak up on this.

The matrix $V$ is described by the 2D numpy array V. To access elements
of this array, we use the syntax

V\[row, col\]

where row is the index of a row and col is the index of a column. For
example, here is how we get the number in the first row (index 0) and
first column (index 0) of V:

``` python
print(V[0, 0])
```

Here is how we get the number in the second row (index one) and first
column (index zero) of V:

``` python
print(V[1, 0])
```

What if we want both of these numbers, so the entire first column? We do
this as follows:

``` python
v1 = V[:, 0]
print(v1)
```

The colon symbol : means \"all\". In particular, V\[:, 0\] means we want
all rows of the first column. Note that the result is a 1D numpy array.

As another example, suppose we want all columns (:) of the second row
(index 1) --- that's easy too:

``` python
print(V[1, :])
```

Let's use this syntax to also find the second eigenvector, the one in
the second column (index 1) of V:

``` python
v2 = V[:, 1]
print(v2)
```

These eigenvalues and eigenvectors are ordered so that the eigenvalue s1
corresponds to the eigenvector v1, and the eigenvalue s2 corresponds to
the eigenvector v2.

Remember that these are eigenvectors and eigenvalues of the matrix $F$.
They should satisfy

$$\begin{aligned}
    Fv_{1}=s_{1}v_{1}\end{aligned}$$

and

$$\begin{aligned}
    Fv_{2}=s_{2}v_{2}.\end{aligned}$$

Let's check:

``` python
print(F @ v1)
print(s1 * v1)

print(F @ v2)
print(s2 * v2)
```

Looks good!

You might be confused at first because, in this case, F @ v1 produces
real (floating-point) numbers and s1 \* v1 produces complex numbers.
However, notice that these complex numbers have zero real part --- -1.
and -1.+0.j are \"the same\" even if one is represented as a real number
and the other as a complex number.

Did you know that there is an easy way to decide if two numpy arrays
have the same values? Here is a little bonus for you careful readers!

``` python
np.allclose(F @ v1, s1 * v1)
```

Wow! See the docs on numpy.allclose for more information. You might be
interested to know that this function is what's being used to do a lot
of the autograding in PrairieLearn.

We have seen that eigenvalues and eigenvectors can be used to
diagonalize a square matrix. In particular, for our 2×2 matrix $F$, we
should find:

$$\begin{aligned}
    diag(s_{1}, s_{2}) = V^{-1}FV\end{aligned}$$

Let's check:

``` python
print(np.diag(s))
print(linalg.inv(V) @ F @ V)
```

Two notes:

-   The function numpy.diag creates a diagonal matrix (if you pass it a 1D array, the elements of this 
    array will be used for the diagonal entries of the matrix).

-   The function scipy.linalg.inv takes the inverse of a square matrix.

Again, you may be confused because (in this case) the first result has
complex numbers but the second result has real numbers. Let's make sure
they are the same:

``` python
np.allclose(np.diag(s), linalg.inv(V) @ F @ V)
```

Our ability to diagonalize a matrix depends on being able to take the
inverse of $V$ --- otherwise, we couldn't evaluate

$$\begin{aligned}
    V^{-1}FV\end{aligned}$$

It is a fact that $V$ will be invertible if all eigenvalues are distinct
(and so all columns of $V$ are linearly independent). That is true in
this case. We could double-check that $V$ is invertible by computing its
determinant using scipy.linalg.det:

``` python
linalg.det(V)
```

Since the determinant $det(V)$ is non-zero, the matrix $V$ is
invertible.

One reason why diagonalization is so helpful is that it is easy to take
the matrix exponential of a diagonal matrix. For example, suppose we
define

$$\begin{aligned}
    S = diag(s_{1}, s_{2}) = \begin{bmatrix}
    s_{1} & 0 \\
    0 & s_{2}
    \end{bmatrix}.\end{aligned}$$

Then we know immediately that

$$\begin{aligned}
    e^{St}=\begin{bmatrix}
    e^{s_{1}t} & 0 \\
    0 & e^{s_{2}t}
    \end{bmatrix}\end{aligned}$$

where the terms $e^{s_{1}t}$ and $e^{s_{2}t}$ are defined in terms of
the scalar exponential function.

In particular, suppose

$$\begin{aligned}
    \dot{x} = Fx.\end{aligned}$$

Then, as we have seen in class,

$$\begin{aligned}
    x(t)=e^{Ft}x(0)=Ve^{St}V^{-1}x(0)\end{aligned}$$

where

$$\begin{aligned}
    S=V^{-1}FV=diag(s_{1}, s_{2}).\end{aligned}$$

That is to say, all of the time-dependent terms in the solution $x(t)$
are scalar exponentials of the eigenvalues of $F$. This, again, is the
basis for our result about asymptotic stability:

The system $\dot{x} = Fx$ is asymptotically stable if and only if all
eigenvalues of $F$ have negative real part.

Let's check that this is true for the following choice of time and of
initial condition (these were chosen arbitrarily as examples):

``` python
# These are just examples - could be any values
t = 0.1
x0 = np.array([[-2.], [1.]])

# Compute a diagonal matrix with eigenvalues in the diagonal
S = np.diag(s)

# Compute solution in two different ways - both should give the same result
print(linalg.expm(F * t) @ x0)
print(V @ linalg.expm(S * t) @ linalg.inv(V) @ x0)
```

Are they really all the same?

``` python
np.allclose(linalg.expm(F * t) @ x0, V @ linalg.expm(S * t) @ linalg.inv(V) @ x0)
```

### What if the eigenvalues are not all distinct

Consider the following matrix:

$$\begin{aligned}
    H = \begin{bmatrix}
    0 & 1 \\
    -4 & 4
    \end{bmatrix}\end{aligned}$$

``` python
H = np.array([[0., 1.], [-4., -4.]])
```

Find the eqigenvalues and eigenvectors of $H$:

``` python
s, V = linalg.eig(H)
```

Are all the eigenvalues distinct?

``` python
print(s)
```

Apparently not. Is $V$ invertible? Let's check the determinant:

``` python
linalg.det(V)
```

It isn't exactly zero, but it's really, really close. We would say that
this is zero to numerical precision. Bottom line, $V$ is not invertible.

Let's see what happens if we try to take the inverse:

``` python
linalg.inv(V)
```

That doesn't look good. Let's see what happens if we try to verify that

$$\begin{aligned}
    diag(s_{1}, s_{2}) = V^{-1}HV\end{aligned}$$

``` python
print(np.diag(s))
print(linalg.inv(V) @ H @ V)
```

Wow, those two things do not look the same. We could make sure:

``` python
np.allclose(np.diag(s), linalg.inv(V) @ H @ V)
```

OK, so what we have discovered is that some matrices cannot be
diagonalized. What then?

In this case, one can compute what is called the Jordan normal form of
the matrix (see also Chapter 6 of Astrom and Murray). We can do this
using SymPy.

``` python
import sympy as sym

H = sym.nsimplify(sym.Matrix(H), rational=True)
V, S = H.jordan_form()
```

Notice that $S$ is still a 2×2 matrix with eigenvalues in the diagonal,
but now there is a 1 in the upper-right corner instead of a 0.

Notice that $V$ is now invertible (and is not a matrix with eigenvectors
in each column any more). We can check it's determinant:

``` python
print(V.det())
```

And, we can take its inverse:

``` python
print(V.inv())
```

We can verify that $V$ and $S$ can be used the same was as before, as if
$H$ were diagonalizable, by checking that $V^{-1}HV=S$:

``` python
V.inv() * H * V
```

Yep, looks good.

And finally, see what happens if we take the matrix exponential of $St$
(remembering to tell SymPy it can assume that $t$ is real-valued):

``` python
t = sym.symbols('t', real=True)
sym.exp(S * t)
```

Cool, right? The scalar exponential terms still have eigenvalues of $H$
in the exponent. The only difference between this and if $S$ were
diagonal is that some of the scalar exponential terms are multiplied by
powers of $t$. Since

$$\begin{aligned}
    e^{-2t}\end{aligned}$$

goes to zero a lot quicker than any power of $t$ gets large, the result
about asymptotic stability still holds:

The system $\dot{x}=Hx$ is asymptotically stable if and only if all
eigenvalues of $H$ have negative real part (even if $H$ is not
diagonalizable!).

## Functions

Start by importing $numpy$ (for matrix manipulation) and $scipy.linalg$
(for computing eigenvalues).

``` python
import numpy as np
from scipy import linalg
```

### The basics

Suppose we want to compute the sum of two particular numbers:

$$\begin{aligned}
2+5\end{aligned}$$

Here is how to do that:

``` python
2 + 5
```

Suppose we want to compute the sum of any two numbers: 

$$\begin{aligned}
    x + y\end{aligned}$$
    
Here is how to do that:

``` python
def compute_sum(x, y):
    return x + y
```

In particular, we have created a function called compute_sum that
takes two arguments called x and y and that returns their sum x + y. We
can now use this function to compute sums:

``` python
compute_sum(2, 5)
```

Note the syntax for defining a function:

- It starts with the keyword def
- After this keyword is the function name
- After the function name, inside parentheses, is a list of
  comma-separated arguments
- After the list of arguments is a colon (and a line break)
- The code inside the function is indented
- The keyword return is used to indicate what value the function will
produce
- There are often many different ways to implement a function. For
example, here is another way to implement the function that computes a
sum:

``` python
def compute_sum(x, y):
    z = x + y
    return z
```

What we have done here is create a variable z inside the function to
hold the result before we return it. Let's check we get the same results
as before:

``` python
print(compute_sum(2, 5))
print(compute_sum(-3, 4))
print(compute_sum(1.234, 5.678))
```

It is important to understand that the variable z --- just like the
other two variables x and y --- are not available outside the function
compute_sum. They exist only in the world of this function.

Let's make this explicit by redefining compute_sum yet again, to print
the value of z inside the function.

``` python
def compute_sum(x, y):
    # Compute the sum
    z = x + y
    # Print the sum
    print(z)
    # Return the sum
    return z
```

First, run compute_sum to verify that we have access to z inside the
function:

``` python
compute_sum(2, 5)
```

Now check if we have access to z outside the function:

``` python
print(z)
```

We should get an error. This goes both ways. Suppose we define two
numbers x and y, then call the function to compute a sum of two
different numbers:

``` python
x = 10
y = 20

compute_sum(2, 5)
```

See what happened there? The function compute_sum completely ignored
the \"global\" values of x and y in favor of the \"local\" values that
were assigned to the function's arguments.

Be careful! Functions in python can make use of variables that are
defined elsewhere. Suppose we had defined compute_sum with different
argument names as follows:

``` python
def compute_sum(a, b):
    return x + y
```

This is clearly a mistake. We called the arguments a and b but returned
x + y. But look:

``` python
compute_sum(2, 5)
```

The reason this happened is that x and y were defined outside the
function --- we had set x = 10 and y = 20. This can cause a lot of
confusion. There are rules for what you can and cannot do inside a
function with variables that are defined outside --- but do yourself a
favor and only use local variables inside functions.

Note that it makes no difference at all what we call these local
variables. For example, here is a correct implementation of
compute_sum with the arguments a and b instead of x and y:

``` python
def compute_sum(a, b):
    return a + b
    
compute_sum(2, 5)
```

### Using functions to make life easier

Functions are great because they allow you to use the same code over and
over but only write that code once.

Consider the problem of deciding whether or not a linear system is
stable. In particular, suppose we want to decide of the system

$$\begin{aligned}
    \dot{x}=Fx\end{aligned}$$

is asymptotically stable, where

$$\begin{aligned}
    F = 
    \begin{bmatrix}
    1 & 3 \\
    -5 & 2
    \end{bmatrix}\end{aligned}$$

We know how to do this. First, we define F:

``` python
F = np.array([[1., 3.], [-5., 2.]])
```

Now, we write the code to check stability:

``` python
# Find eigenvalues of M
s = linalg.eigvals(F)

# Check if all eigenvalues of M have negative real part
(s.real < 0).all()
```

This is a check we will probably do over, and over, and over. It would
be nice to define a function that takes in a square matrix and that
returns True if that matrix defines a stable system, and False
otherwise.

## Ackermann's method

Start by importing the following:

``` python
import numpy as np
from scipy import linalg
from scipy import signal

# Suppress the use of scientific notation when printing small numbers
np.set_printoptions(suppress=True)
```

Consider the application of state feedback 

$$\begin{aligned}
    u=-Kx\end{aligned}$$ 
    
to the open-loop system 
    
$$\begin{aligned}
    \dot{x}=Ax+Bu\end{aligned}$$ 
    
where 

$$\begin{aligned}
A = 
\begin{bmatrix}
-1 & 1 \\
1 & 2
\end{bmatrix}
B = 
\begin{bmatrix}
1 \\
3
\end{bmatrix}\end{aligned}$$ 

As you know, the function
signal.place_poles will find a choice of gain matrix K that puts the
closed-loop eigenvalues anywhere you want (so long as they are not both
in the same location and so long as, if they have a non-zero imaginary
part, the two eigenvalues are a conjugate pair).

In particular, suppose we want to put eigenvalues at −2 and −5 . We
would do this:

``` python
# Define A and B
A = np.array([[-1., 1.], [1., 2.]])
B = np.array([[1.], [3.]])

# Do eigenvalue placement
fbk = signal.place_poles(A, B, [-2., -5.])

# Extract the gain matrix
K = fbk.gain_matrix

# Display the result
print(K)
```

Let's check that it worked:

``` python
print(linalg.eigvals(A - B @ K))
```

### Find the characteristic polynomial that has a given set of roots

Suppose we want to find the coefficients $r_1$ and $r_2$ for which the
polynomial

$$\begin{aligned}
s^2 + r_1s + r_2\end{aligned}$$

would have roots at −2 and −5 . To find these coefficients by hand, we
note that any such polynomial could be factored as

$$\begin{aligned}
    (s - (-2))(s - (-5)) = (s + 2)(s + 5).\end{aligned}$$

Multiplying out, we get

$$\begin{aligned}
    s^2 + r_1s + r_2\end{aligned}$$

Equating coefficients with

$$\begin{aligned}
    s^2 + 7s + 10.\end{aligned}$$

we see that

$$\begin{aligned}
    r_1 = 7 \qquad and \qquad r_2 = 10.\end{aligned}$$

It is easy to automate this process with numpy.poly. Check it out:

``` python
np.poly([-2., -5.])
```

Do you see what happened there? The function np.poly returned a 1d array
with the coefficients of the polynomial that has the given roots. The
first element of this array is the leading coefficient of this
polynomial --- by convention, this coefficient is always 1 . We can
extract the rest of the array as follows:

``` python
r = np.poly([-2., -5.])[1:]
```

The syntax \[1:\] means \"all elements starting at index 1\" (see docs
on numpy indexing). Here is the result:

``` python
print(r)
```

This approach works fine for complex eigenvalues too, so long as they
are in complex conjugate pairs:

``` python
np.poly([-2. + 3.j, -2. - 3.j])
```

You have to be careful though --- suppose there is just a little bit of
numerical imprecision:

``` python
np.poly([-2. + 3.j, -2. - 3.00000001j])
```

Now the coefficients are complex numbers. That's a problem, because no
real-valued choice of $K$ will ever be able to produce a characteristic
polynomial of $A - BK$ with complex coefficients.

To avoid this problem and make our approach more robust to numerical
imprecision, we can simply take the real part of the coefficients:

``` python
np.poly([-2. + 3.j, -2. - 3.00000001j]).real
```

This is something we might as well always do, whether or not the
eigenvalue locations we want are complex --- it will not affect the
result for real eigenvalues, in any case.

``` python
r = np.poly([-2., -5.])[1:].real
print(r)
```

### Find the characteristic polynomial of the open-loop system

The characteristic polynomial of the matrix $A$ is

$$\begin{aligned}
    det(sI-H) = det(s \begin{bmatrix}
    1 & 0 \\
    0 & 1
    \end{bmatrix}-\begin{bmatrix}
    -1 & 1 \\
    1 & 2
    \end{bmatrix}) \\
    = det(\begin{bmatrix}
    s + 1 & -1 \\
    -1 & s - 2
    \end{bmatrix}) \\
    = (s + 1)(s - 2)-(-1)(-1) \\
    =s^{2}-s-3\end{aligned}$$

Again, it is easy to automate this process with numpy.poly. Check it
out:

``` python
np.poly(A)
```

Wow! Apparently, when applied to a matrix (instead of a list), the
function np.poly returns a 1d array with the coefficients of the
characteristic polynomial of that matrix. Just like before, the first
element is the leading coefficient (in this case, the coefficient of
$s^2$ ), which is always 1 . We can extract the rest of the array as
follows:

``` python
a = np.poly(A)[1:]
```

Here is the result:
``` python
print(a)
```

### Find an equivalent system in controllable canonical form

The open-loop system

$$\begin{aligned}
    \dot{z} = A_{ccf}z + B_{ccf}u\end{aligned}$$

is equivalent to the open-loop system

$$\begin{aligned}
    \dot{x} = Ax + Bu\end{aligned}$$

if

$$\begin{aligned}
    A_{ccf} = 
    \begin{bmatrix}
    -a_{1} & -a_{2} & ... & -a_{n-1} & -a_{n} \\
    1 & 0 & ... & 0 & 0 \\
    0 & 1 & ... & 0 & 0 \\
    \vdots & \vdots & \ddots & \vdots & \vdots \\
    0 & 0 & .. & 1 & 0
    \end{bmatrix}
    \qquad
    B_{ccf} =
    \begin{bmatrix}
    1 \\
    0 \\
    0 \\
    \vdots \\
    0
    \end{bmatrix}\end{aligned}$$

where $a_1, ..., a_n$ are the coefficients of the characteristic
polynomial of $A$ . By \"equivalent,\" we mean that $x=Vz$ for some
invertible matrix $V$ .

Since we have already find the coefficients $a_1=-1$ and $a_2=-3$ of the
characteristic polynomial of the particular matrix we have been using in
our running example, we can immediately write

$$\begin{aligned}
    A_{ccf} = \begin{bmatrix}
    1 & 3 \\
    1 & 0
    \end{bmatrix}
    \qquad
    and
    \qquad
    B_{ccf} = \begin{bmatrix}
    1 \\
    0
    \end{bmatrix}\end{aligned}$$

for this example. Guess what, it is easy to automate this process with
numpy.block. This function is exactly the same as np.array, but allows
you to construct a big matrix from smaller matrices rather than from
numbers. Observe:

``` python
Accf = np.block([[-a], [np.eye(1), np.zeros((1, 1))]])
Bccf = np.block([[1.], [np.zeros((1, 1))]])
```

What did we do here?

The matrix $A_{ccf}$ has $-a$ in the top row, the identity matrix (in
this case of size 1×1 , produced by numpy.eye) in the bottom left, and a
column of zeros (in this case of size 1×1 , produced by numpy.zeros) in
the bottom right.

The matrix $B_{ccf}$ has a \"1\" in the top row, and a column of zeros
(in this case of size 1×1 , produced by numpy.zeros) underneath.

Extending this same approach to produce $A_{ccf}$ and $B_{ccf}$ for
systems $(A, B)$ with an arbitrary number of states is only a matter of
changing the sizes of the identity matrix and of the columns of zeros in
the above expressions. Remember that you can find the size of $A$ using
A.shape.

### Design a controller for the equivalent system

If $A_{ccf}$ and $B_{ccf}$ are defined as given above, then it is easy
to show that the gain matrix

$$\begin{aligned}
    K_{ccf} = \begin{bmatrix}
    r_{1} - a_{1} & r_{2} - a_{2} ... & r_{n} - a_{n}
    \end{bmatrix}\end{aligned}$$

puts the eigenvalues of

$$\begin{aligned}
    A_{ccf} - B_{ccf}K_{ccf}\end{aligned}$$

at the roots of

$$\begin{aligned}
    s^{n} + r_{1}s^{n-1} + r_{2}s^{n-2} + ... + r_{n-1}s + r_{n}\end{aligned}$$

Let's compute this gain matrix for our example:

``` python
Kccf = r - a
```

Here is the result:

``` python
print(Kccf)
```

Let's check that it does what it is supposed to do:

``` python
print(linalg.eigvals(Accf - Bccf @ Kccf))
```

Hmm\... that's not good! Something about sizes\... let's check the shape
of $K$:

``` python
print(Kccf.shape)
```

Oh! The problem is that Kccf, as we've defined it, is a 1d array and not
a 2d array like it should be. This is obvious in hindsight, because we
took the difference of two 1d arrays to produce Kccf. This is easy to
fix using numpy.reshape:

``` python
Kccf = (r - a).reshape([1, -1])
```

The function reshape accepts one argument, a list of dimensions. The
first element says the number of rows, in this case \"1\". The second
element says the number of columns --- the \"-1\" means \"however many
it takes\" (sort of like a wild card).

Here is the result:

``` python
print(Kccf)
```

That looks better. Let's try it now:

``` python
print(linalg.eigvals(Accf - Bccf @ Kccf))
```

perfect!

### Find the coordinate transformation between equivalent systems

As you know, if the coordinate transformation

$$\begin{aligned}
    x = Vz\end{aligned}$$

is applied to the system

$$\begin{aligned}
    \dot{x} = Ax + Bu\end{aligned}$$

then the result is the equivalent system

$$\begin{aligned}
    \dot{z} = V^{-1}AVz+V^{-1}Bu\end{aligned}$$

Define the controllability matrix that is associated with A and B as

$$\begin{aligned}
    W = \begin{bmatrix}
    B & AB & A^{2}B .. A^{n-1}B
    \end{bmatrix}.\end{aligned}$$

Since $A$ is n x n and $B$ is n x 1, then $W$ is n x n. Similarly, we
define the controllability matrix that is associated with $A_{ccf}$ and
$B_{ccf}$ as

$$\begin{aligned}
    W_{ccf} = \begin{bmatrix}
    B_{ccf} & A_{ccf}B_{ccf} & A^{2}_{ccf}B_{ccf} .. A^{n-1}_{ccf}B_{ccf}
    \end{bmatrix}\end{aligned}$$

It is a fact that choosing $V$ so that

$$\begin{aligned}
    V^{-1} = W_{ccf}W^{-1}\end{aligned}$$

will establish an equivalence between

$$\begin{aligned}
    \dot{x}=Ax + Bu\end{aligned}$$

and

$$\begin{aligned}
    \dot{z} = A_{ccf}z + B_{ccf}u\end{aligned}$$

That is to say, this choice of $V$ will result in

$$\begin{aligned}
    V^{-1}AV = A_{ccf} 
    \qquad
    and
    \qquad
    V^{-1}B=B_{ccf}\end{aligned}$$

Here is one way to find the controllability matrix $W$ that is
associated with $A$ and $B$ in our example:

``` python
# Find the number of states
n = A.shape[0]

# Initialize W with its first column
W = B

# Create W one column at a time by iterating over i from 1 to n-1
for i in range(1, n):
    col = np.linalg.matrix_power(A, i) @ B
    W = np.block([W, col])
```

The idea is:
- Initialize $W$ with its first column as

$$\begin{aligned}
    \begin{bmatrix}
    B
    \end{bmatrix}\end{aligned}$$

- Iterate from $i=1$ to $i=n-1$, computing the column

    $$\begin{aligned}
        A^{i}B
    \end{aligned}$$

    using numpy.linalg.matrix_power --- sadly, scipy.linalg does not have
    this function --- and then appending this column to $W$ using
    numpy.block, which you've seen before

We can do exactly the same thing to find $W_{ccf}$:

``` python
# Initialize Wccf with its first column
Wccf = Bccf

# Create W one column at a time by iterating over i from 1 to n-1
for i in range(1, n):
    col = np.linalg.matrix_power(Accf, i) @ Bccf
    Wccf = np.block([Wccf, col])
```

(This is, of course, begging for you to wrap it in a function.)

And now, it is a simple matter to find $V^{-1}$:

``` python
inverse_of_V = Wccf @ linalg.inv(W)
```

We are finding $V^{-1}$ instead of $V$ because it is the inverse that we
will need later.

Does it actually work? Remember, we want

$$\begin{aligned}
    V^{-1}AV=A_{ccf}
    \qquad
    and
    \qquad
    V^{-1}B=B_{ccf}.\end{aligned}$$

Let's check:

``` python
print(inverse_of_V @ A @ linalg.inv(inverse_of_V))
print(Accf)

print(inverse_of_V @ B)
print(Bccf)
```

Apparently it does!

### Find equivalent state feedback

If

$$\begin{aligned}
    x = Vz\end{aligned}$$

then

$$\begin{aligned}
    u = -K_{ccf}z = -K_{ccf}(V^{-1}x)=-(K_{ccf}V^{-1})x.\end{aligned}$$

For what gain matrix K would

$$\begin{aligned}
    u=-Kx\end{aligned}$$

be equivalent to this choice of input? Obviously this one (by equating
coefficients):

$$\begin{aligned}
    K=K_{ccf}V^{-1}\end{aligned}$$

Let's find it, for our example:

``` python
K = Kccf @ inverse_of_V
```

Does it work? In particular, does it put the eigenvalues of $A-BK$ at −2
and −5 , like we wanted? Let's check:

``` python
print(linalg.eigvals(A - B @ K))
```

Yep! How lovely.

### Finish up and test

Your implementation of acker is now complete. Test it:

``` python
K = acker(A, B, [-2., -5.])
print(K)
print(linalg.eigvals(A - B @ K))
```

Does it work for complex eigenvalues?

``` python
K = acker(A, B, [-5. + 10.j, -5. - 10.j])
print(K)
print(linalg.eigvals(A - B @ K))
```

What about for eigenvalues at the same location?

``` python
K = acker(A, B, [-3., -3.])
print(K)
print(linalg.eigvals(A - B @ K))
```

Wow, not even signal.place_poles does that! Observe:

``` python
fbk = signal.place_poles(A, B, [-3., -3.])
```

Note, however, that there can be some numerical error when you ask for
eigenvalues at the same location:

``` python
K = acker(A, B, [-1., -1.])
print(K)
print(linalg.eigvals(A - B @ K))
```

This is a clue about why place_poles does not allow you to ask for
this.

## LQR

Start by importing the following:

``` python
import numpy as np
from scipy import linalg
```

This is the infinite-horizon Linear Quadratic Regulator (LQR) optimal
control problem: 

$$\begin{aligned}
\mathop{\mathrm{minimize}}_{x(t_{1}),n_{(- \infty,t_{1}]},d_{(- \infty,t_{1}]}}
&\qquad
\int_{- \infty}^{t_{1}} \left( n(t)^{T}Q_{o}n(t)+d(t)^{T}R_{o}d(t) \right) dt \\
\text{subject to}
&\qquad
\dot{x}(t) = Ax(t)+Bu(t)+d(t) \qquad for \qquad t \epsilon (- \infty, t_{1}]\\
&\qquad
y(t) = Cx(t)+n(t)\end{aligned}$$

The integrand is cost. The integral is total cost. The square matrices Q
and R are weights --- you, as a control designer, get to choose these
weights. The minimizer (i.e., the input that achieves minimum total
cost) is

$$\begin{aligned}
    u(t)=-Kx(t)\end{aligned}$$

and the minimum (i.e., the minimum total cost) is

$$\begin{aligned}
    x_{0}^{T}Px_{0}\end{aligned}$$

where P and K can be found in python as follows:

``` python
P = linalg.solve_continuous_are(A, B, Q, R)
K = linalg.inv(R) @  B.T @ P
```

There are a number of reasons why LQR is a good choice for control
design. At this point, we will simply say that LQR makes it easy to get
a working controller.

Why? Notice that, for this optimal control problem to make any sense,
its minimum total cost --- i.e., the value of the integral for the
\"best\" choice of K --- has to be finite. Since the upper limit of the
integral is infinity, then --- in order for the integral to be finite
--- the integrand has to converge to zero (quickly). The only way for it
to do that is if the closed-loop system is asymptotically stable. In
other words, the \"best\" choice of has to produce a closed-loop system
that is asymptotically stable.

Put simply, the solution to the LQR problem, for any choice of weights,
is a controller that produces a stable closed-loop system --- i.e., a
\"working controller.\"

Let's look at an example.

Suppose we want to design a controller by solving the LQR problem for
the system described by

$$\begin{aligned}
A = 
\begin{bmatrix}
0 & 1 \\
0 & 0
\end{bmatrix}
B = 
\begin{bmatrix}
0 \\
1
\end{bmatrix}\end{aligned}$$

and for weights that are both identity matrices: 

$$\begin{aligned}
Q = I_{2x2} =
\begin{bmatrix}
1 & 0 \\
0 & 1
\end{bmatrix}
R = I_{1x1}
\begin{bmatrix}
1
\end{bmatrix}\end{aligned}$$

First, we define the system:

``` python
A = np.array([[0., 1.], [0., 0.]])
print(f'A = {A.tolist()}')

B = np.array([[0.], [1.]])
print(f'B = {B.tolist()}')
```

Second, we define the weights:

``` python
Q = np.eye(2)
print(f'Q = {Q.tolist()}')

R = np.eye(1)
print(f'R = {R.tolist()}')
```

Third, we find the optimal cost matrix:

``` python
P = linalg.solve_continuous_are(A, B, Q, R)
print(f'P = {P.tolist()}')
```

Fourth, and finally, we find the optimal gain matrix:

``` python
K = linalg.inv(R) @  B.T @ P
print(f'K = {K.tolist()}')
```

As we claimed, this gain matrix makes the closed-loop system stable:

``` python
linalg.eigvals(A - B @ K)
```

Suppose we start with the following initial condition: 

$$\begin{aligned}
    x_{0}=
    \begin{bmatrix}
    1\\
    -2
    \end{bmatrix}\end{aligned}$$

``` python
x0 = np.array([1., -2.])
```

The input that we would apply at the time $t_0$ is, of course, the
following: 

$$\begin{aligned}
    u(t_{0})=-Kx(t_{0})=-Kx_{0}\end{aligned}$$ 
    
Let's compute it:

``` python
u = - K @ x0
print(f'u = {u.tolist()}')
```

Let's also compute the total cost 

$$\begin{aligned}
    x_{0}^{T}Px_{0}\end{aligned}$$ 
    
that we would accumulate if we continued to apply this same controller 

$$\begin{aligned}
    u(t)=-Kx(t)\end{aligned}$$ 
    
for all time, starting from

$$\begin{aligned}
    x(t_{0})=x_{0}\end{aligned}$$

``` python
total_cost = x0.T @ P @ x0
print(f'total_cost = {total_cost}')
```

Note that the total cost is a scalar quantity (just one real number, not
a vector or matrix).

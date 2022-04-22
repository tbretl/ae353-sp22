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


# Observers
T. Bretl, S. Bout, J. Virklan

## What is an observer?

Consider the usual state space system 

$$\begin{align*}
    \dot{x} = Ax + Bu
\end{align*}$$ 
        
The controller $u = -Kx$
assumes complete knowledge of the state $x$. In reality, this is almost
never the case. In the scenario where complete knowledge of the state is
not known, an observer is used. An observer is the \"best guess\", or
the best estimate of the state based on the information available. The
state estimate is denoted as $\hat{x}$, and the choice of input $u$ will
be based on the state estimate instead of the state. The controller

$$\begin{aligned}
    u = -Kx
\end{aligned}$$ 
        
becomes 

$$\begin{aligned}
    u = -K \hat{x}
\end{aligned}$$ 

The primary source of the
information used to determine $\hat{x}$ comes from sensors. If these
measurements from the sensors are denoted as $y$, then the goal is to
somehow go from these measurements $y$, to the state $\hat{x}$. The
relationship between the measurements and the state can be formulated as

$$\begin{aligned}
    y = C \hat{x}
\end{aligned}$$ 

An approach we might consider is

$$\begin{aligned}
    \hat{x}(t) = C^{-1}y(t)
\end{aligned}$$ 

The reason this will not
generally work is because the $C$ matrix may not be square or
invertible.\
The approach that will generally work, is an observer. An observer is
defined as a set of ordinary differential equations that look very
similar to the original state space model, but has $\hat{x}$ instead of
$x$. A corrective term that is proportional to $C\hat{x} - y$ is added.
The resulting formula is called an observer 

$$\begin{aligned}
    \dot{\hat{x}} = A \hat{x} + Bu - L(C \hat{x} - y)
\end{aligned}$$

If the measurements $y$ and the inputs $u$ are plugged in, and
$\dot{\hat{x}}$ is integrated, this will produce a very good state
estimate which will allow for performance similar to control applied to
the exact value of the state itself.

## Do observers make sense?

So far, we have shown that if we integrate the observer

$$\begin{aligned}
    \dot{\hat{x}} = A\hat{x} + Bu - L(C \hat{x} - y)
\end{aligned}$$

it will produce an estimate of the state of the system 

$$\begin{aligned}
    \dot{x} = Ax + Bu \\
    y = Cx
\end{aligned}$$ 

Why is this the case? And where does this
expression for an observer come from? To understand this, we must look
back to how controllers are designed. In the absence of a controller
where there is no input, the state would behave as 

$$\begin{aligned}
    \dot{x} = Ax
\end{aligned}$$ 

If we want the system to behave in a
different manner, such as to achieve a stable system where it was
unstable, we add an expression that is negatively proportional to the
error 

$$\begin{aligned}
    \dot{x} = Ax - BK(x - x_{desired})
\end{aligned}$$

where error is
the difference between the actual state, and the desired state. In the
case of control, the desired state is 0, at least in the context of
state feedback to generate the usual expression for a closed-loop system

$$\begin{aligned}
    \dot{x} = (A - BK)x 
\end{aligned}$$ 

What about in the case of
state estimation? In the absence of any other information, the best
guess for the state is produced by integrating the state space model
from some initial condition 

$$\begin{aligned}
    \dot{x} = A\hat{x} + Bu, \; \hat{x}(0) = x_{0}
\end{aligned}$$ 

If
the guess at the initial condition is perfect, and the state space model
is perfect, than integrating (1) would produce an exact estimate of the
state. In reality, the estimate of the initial condition and the model
are never perfect. One way to address that problem is to add a term to
the state space model that is negatively proportional to the error, the
difference between the state estimate and the actual state, resulting in

$$\begin{aligned}
    \dot{\hat{x}} = A\hat{x} + Bu - L(\hat{x} - x)
\end{aligned}$$

This is something that cannot be implemented because in order to
implement it, we would have to know the value of $x$, which of course we
do not know because that is the whole point of finding the estimate of
the state $x$. However, we can do something similar, where a term is
added to the state space model that is negatively proportional to some
measure of error, but instead of the measure of error between the state
estimate and the actual state, which cannot be implemented, the term
added is negatively proportional to the difference between what the
model tells us our measurement would be, $C\hat{x}$ if $\hat{x}$ were
the state, and what the measurement actually is $y$. The resulting
observer is 

$$\begin{aligned}
    \dot{\hat{x}} = A\hat{x} + Bu - L(C \hat{x} - y)
\end{aligned}$$

where $C \hat{x} - y$ is the error in prediction of the measurement $y$.
So we see just like in the case of control, we take some nominal
behaviour and correct it by adding a term that is negatively
proportional to some measure of error. Observers make sense!

## When does an observer work?

One way to decide if an observer is going to work is to predict what the
error in the observer measurements is, that is to say: 

$$\begin{aligned}
    e &= (x - \hat{x})\\
    x &= Ax + Bu\\
    \dot{\hat{x}} &= A\hat{x} + Bu - L(Cx - C\hat{x})
\end{aligned}$$

subtracting the 2nd and 3rd equation above yields an equation that can be
described purely in terms of the error and its time derivative, that is:

$$\begin{aligned}
    \dot{e} = (A-LC)e
\end{aligned}$$ 

The error equation is similar to
the case for controller design where $\dot{x} = (A-BK)x$, and the system
can be analyzed for asymptotic stability in the same way. By defining
the characteristic equation: 

$$\begin{aligned}
    0 = det(sI - (A-LC))
\end{aligned}$$ 

the solution to this equation is
the eigenvalues of $A-LC$, and similar to control design, if the real
parts of those eigenvalues are all negative, the system will be
asymptotically stable through time.

## How to choose L for an observer

The fact that negative real parts of the eigenvalues of $A-LC$ leads to
a stable system leads one to desire a method similar to Ackerman's
method for controller design. Approaching the problem in this context,
we desire to place poles for the system such that stability is achieved
in the observer. The issue in the equations given and applying those to
Ackerman's method is that the scripts developed for Ackerman's method
take specific sized matrices to translate to a properly sized K matrix.
We can resolve these issues by transposing the characteristic equation
of $A-LC$ yielding $A^T - C^TL^T$, and now the system looks exactly like
that of $A-BK$, but transposed to fit properly. When placing this into
the acker() algorithm, this yields $L^T =$ acker($A^T$, $C^T$,
$p_{obs}$) where $p_{obs}$ are the desired poles to place. We want L and
not the transpose of L, but this can be resolved by taking the
transpose of acker().

## Do observers break controllers

Given the overarching model with the controller and observer design
implemented together, its important that we check that for any observer
designed separately from the controller in this context maintains
overall system stability. we know that: 

$$\begin{aligned}
    x & = A + Bu\\
    y & = Cx\\
    u &= -Kx\\
    \dot{\hat{x}} &= A\hat{x} + Bu - L(y-C\hat{x})
\end{aligned}$$

Combining these equations together, we can get: 

$$\begin{aligned}
    \begin{bmatrix}
    \dot{x} \\
    \dot{e}
    \end{bmatrix}
    =
    \begin{bmatrix}
    [A-BK] & [-BK]\\
    [0] & [A-LC]
    \end{bmatrix}
    \begin{bmatrix}
    x \\
    e
    \end{bmatrix}
    =
    [H]q
\end{aligned}$$ 

with \[0\] being an appropriately sized matrix
to make the overall structure of the block matrix square. Taking the
roots of the block matrix yields: 

$$\begin{aligned}
    0 = det(sI - H)
\end{aligned}$$ 

One important concept from linear
algebra has to deal with manipulation of block matrices within
determinants, that rule is: 

$$\begin{aligned}
    det(\begin{bmatrix}
    F & E \\
    0 & G
    \end{bmatrix} = det(F)det(G)
\end{aligned}$$ 

applying this to the
block matrix H yields: 

$$\begin{aligned}
    det(sI-H) = det(sI-(A-BK))det(sI-(A-LC))
\end{aligned}$$ 

The roots of
both determinants are given by the eigenvalues of their respective
matrices. This means that if both determinants present an asymptotically
stable system, then the overarching system will remain stable in time.
This gives rise to the Separation Principle, that is, one can design an
observer and controller separately while maintaining the stability of
each and merge them together while and still achieve overall stability.
We can design the controller and observer while ignoring that the other
even exists.

## When is observer design possible

We know of the controllability matrix from controller design
$W_c = [B AB ... A^{n-1}B]$ which tells whether the system is
controllable given as the controllability matrix being of full rank.
Similarly, we can construct an observability matrix as:

$$\begin{aligned}
    W_o = \begin{bmatrix}
    C^{T} &
    A^{T} C^{T} &
    . . .  &
    A^{n-1T} C^T
    \end{bmatrix}
\end{aligned}$$

Wishing to eliminate the messiness of the transposes, we rewrite $W_o$
as: 

$$\begin{aligned}
    W_o^T = \begin{bmatrix}
        C \\
    CA\\
    \vdots \\
    CA^{n-1}
    \end{bmatrix}
\end{aligned}$$ 

similar to the control problem, the
system is observable if $W_o^T$ is of full rank. One should always check
the observability of a system before trying to design an observer for
it.

# Optimal observers

## What is an optimal observer?

Consider the usual state space system 

$$\begin{aligned}
    \dot{x} = Ax + Bu \\ 
    y = Cx
\end{aligned}$$ 

let us denote $n_{x}$ as the number of
states in that system, $n_{u}$ as the number of inputs, and $n_{y}$ as
the number of outputs. The optimal controller 

$$\begin{aligned}
    u = -Kx
\end{aligned}$$ 

where $K$ can be found using the Python LQR()
function we defined earlier in the \"LQR Code\" section under \"LQR\" in
the \"Optimization and Optimal Control\" section. 

```
K=LQR(A, B, Qc, Rc)
```

$Q_{c}$ is size $n_x$ x
$n_x$ and $R_{c}$ is size $n_{u}$ x $n_{u}$. We know these matrices have
to satisfy certain properties. In particular, $Q_{c}$ must be positive
semidefinite, $R_{c}$ must be positive definite. An easy way to ensure
these properties are satisfied is to choose both of these matrices as
diagonal with positive numbers (i.e. identity matrix). What optimal
means is that the choice of $K$ is making the best trade-off it can
between error, which is being penalized by $Q_{c}$, and effort, which is
being penalized by $R_{c}$. If we increase the numbers in the diagonal
of either matrix, the resulting $K$ is going to be making a different
trade-off between error and effort depending on those numbers. The same
approach can be used to design an optimal observer. Any observer has the
form 

$$\begin{aligned}
    \dot{\hat{x}} = A\hat{x} + Bu - L(C \hat{x} - y)
\end{aligned}$$ 

An
optimal observer is where L is chosen using the Python function LQR

```
L = LQR(A.T, B.T, linalg.inv(Qo), linalg.inv(Ro)).T
```

Notice $A$ and $B$ are transposed, and that $Q_{o}$ and $R_{o}$ are
inverted. This LQR function produced the transpose of $L$, so we need to
take the transpose to get our final $L$. $Q_{o}$ is size $n_y$ x $n_y$
and $R_{o}$ is size $n_{x}$ x $n_{x}$. The matrices need to satisfy
certain properties. An easy way to ensure this is to choose diagonal
matrices with positive numbers. An identity matrix is a good place to
start. $Q_{o}$ penalizes sensor noise, and $R_{o}$ penalized process
noise. Increasing the numbers in $Q_{o}$ means we trust the sensors more
because we are penalizing the noise that we are allowing the observer
design process to assume is happening. Similarly, increasing the numbers
in $R_{o}$, we are trusting our state space model more, and
de-emphasizing the information we are getting from the sensors. The nice
part about this design is that LQR can be used to design both the
controller and the optimal observer.

## What problem is solved to produce an optimal observer?

This is the optimal control problem that is solved to produce an optimal
controller 

$$\begin{aligned}
\mathop{\mathrm{minimize}}_{u_[t_0, \infty)}
&\qquad
\int_{t_{0}}^{\infty} \left( x(t)^{T}Q_{c}x(t)+u(t)^{T}R_{c}u(t) \right) dt \\
\text{subject to}
&\qquad
\dot{x}(t) = Ax(t)+Bu(t) \qquad \text{for} \qquad t \epsilon [t_0, \infty] \\
&\qquad
x(t_0) = x_0
\end{aligned}$$

. This optimal controller
has the form 

$$\begin{aligned}
    u = -Kx
\end{aligned}$$ 

where K can be found using the LQR() Python
function we previously defined. This is the optimal control problem that
is solved to produce an optimal observer 

$$\begin{aligned}
\mathop{\mathrm{minimize}}_{x(t_{1}),n_{(- \infty,t_{1}]},d_{(- \infty,t_{1}]}}
&\qquad
\int_{- \infty}^{t_{1}} \left( n(t)^{T}Q_{o}n(t)+d(t)^{T}R_{o}d(t) \right) dt \\
\text{subject to}
&\qquad
\dot{x}(t) = Ax(t)+Bu(t)+d(t) \qquad for \qquad t \epsilon (- \infty, t_{1}]\\
&\qquad
y(t) = Cx(t)+n(t)\end{aligned}$$

Here again, the solution
involves using a gain matrix, in this case we call this matrix L, which
can also be found using the LQR() Python function we previously defined.
L in this case enters into a set of ordinary differential equations

$$\begin{aligned}
    \dot{\hat{x}} = A\hat{x}+Bu-L(c\hat{x}-y)
\end{aligned}$$ 

Let's take
a look at some differences between these two problems. In the optimal
control problem, the term, $x(t)^TQ_c x(t)$, penalizes error. The goal
of the controller is to drive $x$ to zero, and when that is not zero,
this error grows quadratically. The term $u(t)^TR_c u(t)$ penalizes
effort quadratically as well, but this time on the choice of input $u$.
Now if we take a look at our optimal observer problem, instead of
penalizing error and effort, the term $n(t)^T Q_o n(t)$ penalizes sensor
noise and the term $d(t)^T R_o d(t)$ penalizes the process disturbance.
These variables $n(t)$ and $d(t)$ are variables we are adding to the
state space model to capture real world faults. The disturbance $d(t)$
captures how the inputs we apply are always corrupted in some way in the
real world. We can see this 

$$\begin{aligned}
    d(t) = \dot{x}(t) - (Ax(t)+Bu(t)
\end{aligned}$$ 

from this we see
$d(t)$ captures the error in our model of the dynamics. If our model of
the dynamics was perfect, $\dot{x}(t) = (Ax(t) + Bu(t)$. The fact that
it is not, is represented by $d(t)$. The term $d(t)^T R_o d(t)$
recognizes this an imposes a quadratic penalty on the extent to which
our model of the dynamics is wrong. Similarly, $n(t)$ captures the
extent to which our sensor measurements are wrong in the real world.

$$\begin{aligned}
    -n(t) = Cx(t) - y(t)
\end{aligned}$$ 

We see again, similar to
disturbance, this is the error in our prediction of what the measurement
should be. So, while the optimal controller quadratically penalizes
error and effort, the optimal observer quadratically penalizes the
sensor noise and process disturbance. For the optimal controller, we
integrate from the current time $t_0$ to the end of time $\infty$. For
the optimal observer, we integrate from the beginning of time $-\infty$
of the system, to the current time. In particular, for the optimal
controller, the value we are fundamentally trying to find, is the choice
of input $u$, while for the optimal observer, the value we are
fundamentally trying to find, is the current state. We might notice that
there is given current state $x(t_0)$ in the optimal observer as we see
in the optimal controller. This is the whole point of an optimal
observer. The optimal observer gives the best estimate of the current
state, which allows the controller to give the best inputs based on this
reported current state. The two complement each other.

We see that the optimal observer also minimizes over the noise and the
disturbance. This is because we want the optimal observer to also
explain the best estimate it has arrived to. The noise and the
disturbance effectively explain the differences between the expected and
actual measurements and model dynamics respectively. The last difference
between the optimal controller and optimal observer is a bit more
subtle. The final result of the optimal controller is the formula

$$\begin{aligned}
u = -Kx
\end{aligned}$$ 

The optimal observer does not have such a
declarative formula. While there is a similar expression for state
estimate, the formula would be dependent on inputs and measurements that
have been chosen and observed respectively from the beginning of time
$-\infty$. This means that in order to evaluate this expression, we
would need to keep track of every single input and measurement since the
beginning of time of that system. If that sounds ridiculous, it's
because it is. The best we can do is an expression that explains how the
observer evolves over time in the form of the ordinary differential
equation 

$$\begin{align*}
\tag{1}
    \dot{\hat{x}} = A\hat{x} + Bu - L (C\hat{x} - y)
\end{align*}$$

Suppose we were to solve the optimal observer problem, we notice it has
an upper limit of integration $t_1$ which we consider the \"current
time\", it produces an estimate of the current state $x(t_1)$. For
example, if we solved the optimal observer problem such that
$t_1 = t_\alpha$, then $x(t_1) = \alpha$, and we solved it again such
that $t_1 = t_\beta$, then $x(t_1) = \beta$. If we integrate (1)
starting at $\hat{x}(t_\alpha) = \alpha$ then the solution is
$\hat{x}(t_\beta) = \beta$. This can effectively be considered an update
expression. It allows us to go from a solution at one time of the
observer problem, and produces a new solution at another time, without
having to track all the inputs and measurements from the beginning of
time. Instead, we only need to know the inputs and measurements from
time $t_\alpha$ to time $t_\beta$ in this scenario. This is why we see
an explicit formula for the optimal controller, and a more subtle
\"update expression\" for the optimal observer.

## Do optimal observers make any sense at all?

The equations that govern optimal observers may appear confusing. Using
a simple model 

$$\begin{aligned}
    y = x
\end{aligned}$$

 and a simple sensor measurement $y = 10$ it is
safe to assume the state estimate $\hat{x} = 10$. Now assume there are
two sensors $y_1 = 10$ and $y_2 = 20$. The state estimate is no longer
so easy. Some might consider $\hat{x} = 15$, an average between the two.
Let's say for example, these sensor measurements record altitude. $y_1$
measures the value from an altimeter, and $y_2$ measures the value from
a laser range finder. Those with some familiarity of these two sensors
will know that a laser range finder tends to be significantly more
accurate, so $\hat{x} = 15$ is not really a reasonable choice of state
estimate. Instead, we would prefer to choose a value much closer to the
more accurate sensor, in this scenario the laser range finder. For
example, we would decide $\hat{x} = 19$ instead, to reflect the higher
accuracy of the laser range finder over the altimeter. This could still
be described as an average, but it is now a weighted average. In this
scenario 

$$\begin{align*}
\tag{2}
    \hat{x} = 0.1y_1 + 0.9y_2
\end{align*}$$ 

This raises the question:
is there a best, or optimal, choice of weights for this particular case,
or for a more general case? Hint: the answer is yes. One way to see that
would be to minimize the cost such that we can derive the weighted
average as a consequence. One way to do that would be to extend the
sensor models. Suppose we add a term to each sensor 

$$\begin{aligned}
    y_1=x+n_1 \\
    y_2 = x + n_2
\end{aligned}$$

 where $y_1$ is the altimeter, $y_2$ is
the laser range finder, and the added terms $n_1$ and $n_2$ is the noise
of the sensor. Sensor noise is a way of acknowledging that real world
sensor measurements are never perfect and explicitly modeling how much
noise is being added to the state in order to produce each measurement.
For any given guess at what the state is, we can quantify how much noise
must have been added to produce the first measurement, and how much
noise must have been added to produce the second measurement. For
example, if we assume the actual state $x = 19$, and the altimeter
measures $\hat{x} = 10$, then the measurement from the altimeter, $y_1$,

$$\begin{aligned}
    n_1 = y_1 - x \\
    -9 = 10 - 19
\end{aligned}$$

 is off by a value of -9, and the
measurements from the laser range finder measure $\hat{x} = 20$, then
$y_2$ 

$$\begin{aligned}
    n_2 = y_2 - x \\
    1 = 20 - 19
\end{aligned}$$ 

is only off by a value of 1. A trade-off
occurs between the assumed noise of the two measurements, in this case
the altimeter and the laser range finder, in order to explain the
measurements we see for a given choice of the actual state. The best
choice of state estimate, a choice that minimizes the noise of both
measurements. Here is the cost that quantifies that. We want to minimize
over all possible choices of $x$, $n_1$, and $n_2$: 

$$\begin{aligned}
\mathop{\mathrm{minimize}}_{x, n_1, n_2}
&\qquad
q_1(n_1)^2 + q_2(n_2)^2 \\
\text{subject to}
&\qquad
y_1 = x + n_1\\
&\qquad
y_2 = x+n_2\end{aligned}$$

where $q_1$ and $q_2$ are
weights. So for a state estimate $x$ to be good, we have to minimize the
sum squared of the noise from the first and second sensor. This
optimization problem is easily simplified and solved. The first step is
to replace the noise variables with the measurements and state
equivalent: 

$$\begin{aligned}
\mathop{\mathrm{minimize}}_{x}
&\qquad
q_1(y_1-x)^2 + q_2(y_2-x)^2
\end{aligned}$$

Let's call this function $f(x)$. This allows us to eliminate our constraints and the ends to our
minimization. We can now recognize that this function we are minimizing
is a single scalar valued function of $x$. We know how to minimize
single scalar valued functions. We set the first derivative of that
function to zero 

$$\begin{aligned}
    0 = \frac{df}{dx} = -2q_1(y_1-x) - 2q_2(y_2-x) \\
    = -2 (q_1(y_1-x)+q_2(y_2-x)) \\
    =-2((q_1 y_1 + q_2 y_2) - (q_1 + q_2) x)
\end{aligned}$$ 

which
ultimately leads to 

$$\begin{aligned}
    x = (\frac{q_1}{q_1+q_2})y_1 + (\frac{q_2}{q_1+q_2})y_2 
\end{aligned}$$

A couple things to note. We have derived the same weighted expression
earlier as shown in (2). This means taking a weighted average is
equivalent to minimizing a sum squared cost on the amount of noise we
would have to assume to explain the measurements we see. But how do we
choose these weights? Choosing $q_1$ and $q_2$ is quite easy in
practice. The right way to choose these weights is to take

$$\begin{align*}
\tag{3}
    \frac{1}{\sigma^2}
\end{align*}$$ 

where $\sigma$ is the associated
standard deviation with a given sensor. This means, if we assume the
noise in the laser range finder is normally distributed with a zero mean
and a standard deviation $\sigma_2$, then the right way to take the
weight $q_2$ is by using equation (3). Looking back at the optimal observer
function 

$$\begin{aligned}
\mathop{\mathrm{minimize}}_{x(t_{1}),n_{(- \infty,t_{1}]},d_{(- \infty,t_{1}]}}
&\qquad
\int_{- \infty}^{t_{1}} \left( n(t)^{T}Q_{o}n(t)+d(t)^{T}R_{o}d(t) \right) dt \\
\text{subject to}
&\qquad
\dot{x}(t) = Ax(t)+Bu(t)+d(t) \qquad for \qquad t \epsilon (- \infty, t_{1}]\\
&\qquad
y(t) = Cx(t)+n(t)\end{aligned}$$


 we can see we are minimizing
over all possible choices of state estimate, the sum squared of the
noise $n(t)$ and the disturbance $d(t)$ that would have to be assumed in
order to explain the differences we observe between $\dot{x}(t)$ and
$Ax(t) + Bu(t)$ and the differences we observe between $y(t)$ and
$Cx(t)$. In exactly the same way that in order to derive the weighted
average to combine the measurements between the altimeter and the laser
range finder in our example, we minimized the weighted sum squared of
the noises that we would have to assume to explain the differences
between the measurements and the state. A note: While taking the inverse
squared standard deviation to find the optimal weights for $q_1$ and
$q_2$, we can apply this to the matrices $Q_o$ and $R_o$ such that

$$\begin{aligned}
    Q_o = \sum_{n}^{-1} \\
    R_o = \sum_{d}^{-1}
\end{aligned}$$ 

where $\sum_{n}^{-1}$ is the
inverse of the covariance matrix that is associated with the noise $n$
and $\sum_{d}^{-1}$ is the inverse of the covariance matrix that is
associated with the disturbance $d$. We can see it is advantageous to
use this method rather than a simple weighted average because there is a
direct way to find the weights to minimize the observer function.
Ultimately, all an optimal observer is doing, is computing a more
complicated weighted average.

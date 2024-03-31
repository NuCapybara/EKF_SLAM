 Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- geometry2d - Handles 2D geometry primitives
- se2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. If you needed to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality 
   - General Idea: Calculate the magnitude of the vector based on vector x and y, and then use original x and y to divide the magnitude.
   - Method 1: Make it member function of a Vector2D struct. 
   
        PRO:  statically type safe (Guidelines P.4), Express ideas directly in code(Guidellines P.1), Make a function a member only if it needs direct access to the representation of a class(Guidelines C.4)

        CON: It will directly changes the state of the vector object, could bring inconvinience. (Guidelines C.65)
   ```
   struct Vector2D {}
    double x, y;

    void normalize() {}
    ```

   - Method 2: Make it Non-member function and return a Vector2D. 

        PRO: Express ideas directly in code(Guidellines P.1).Does not alter the original vector(Guidelines C.65). Can be used with different types if overloaded appropriately.
        
        CON: 
        Less intuitive because it's outside from the class. (Guidellines P.1)
   ``` 
   Vector2D normalize(const Vector2D& vec)
   ```

   - Method 3: Make it friend function of the Vector2D struct.

        PRO: Access to Private Members. Clear Intent.(Guidellines P.1)

        CON: 
        Less Common in Structs.
   ```
   struct Vector2D {
    double x, y;
    friend Vector2D normalize(const Vector2D& vec);
    };
   ```

   - Which of the methods would you implement and why?

    I would do the second one for avoiding changing the original vector.

2. What is the difference between a class and a struct in C++?

    Class: The default access in a class is private.And class also provides users to "public", "private", "protected".  And user are able to implement more complex data structures in Class.

    Struct: The default access in a struct is public. User can implement simpler and passive data structure.

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?
    C2: : Use class if the class has an invariant; use struct if the data members can vary independently.

    C.8: Use class rather than struct if any member is non-public

    C.9: Minimize exposure of members

    Because Transform2D has so many private members, a user cannot initialize the object without use of constructor. Hence a class definer could provide constructor for that but struct cannot. The user needs to define an invariant. But for Vector2D, the members can vary independently and be public, so we use struct. Besides, the class structure minimize the exposure of members.

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

    C.46: By default, declare single-argument constructors explicit
    Because some Transform2D only took one argument and those are explicit to avoid unintended conversions.

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

   Con.2: By default, make member functions const. A member function should be marked const unless it changes the object’s observable state. This gives a more precise statement of design intent, better readability, more errors caught by the compiler, and sometimes more optimization opportunities.

    According to the above statement, inv() is not changing object's observable state. but operator*= is changing due to the multiplication to the right hand side.
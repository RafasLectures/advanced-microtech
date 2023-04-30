/*****************************************************************************
 * @file                    helpers.hpp
 * @author                  Rafael Andrioli Bauer
 * @date                    09.11.2022
 * @matriculation number    5163344
 * @e-mail contact          abauer.rafael@gmail.com
 *
 * @brief   Header that provides some helper methods to manipulate the registers
 *
 * @note    The project was exported using CCS 12.1.0.00007
 ******************************************************************************/

#ifndef MICROTECH_HELPERS_HPP
#define MICROTECH_HELPERS_HPP

#include <cstdint>

/**
 * Method to set the desired bits of a register to 1.
 * @param registerRef is the reference to the register to be accessed.
 *                    (the reference is like a const pointer, so we are accessing directly the
 *                    register here and not copying its value. One can think of it as const pointer
 *                    and when accessing this parameter inside of this function
 *                    the compiler automatically applies the * operator on the pointer.
 *                    For further explanation, I found this website
 *                    (https://www.geeksforgeeks.org/pointers-vs-references-cpp/#:~:text=Pointers%3A%20A%20pointer%20is%20a,for%20an%20already%20existing%20variable.)
 * @param bitSelection is the bits in which you want to set. E.g if you want to set bit4 of the register P1DIR you would
 * call:
 *                     @code
 *                        setRegisterBits(P1DIR, 0x10);
 *                     @endcode
 * @tparam TYPE to enforce that one is setting the bitSelection according to the register size. More info bellow.
 *
 * The template was added so the the compiler can also do type checking to avoid unwanted behavior.
 * Like when you try to set a 16bit register with an 8bit bitMask, for example.
 *
 * One doesn't necessarily needs to define the template parameter, like setRegister<uint8_t>(P1DIR, 0x10),
 * the compiler is smart enough to deduce the template argument TYPE when using this function. So the compiler
 * enforces that both parameters are of the same size, like uint8_t or uin16_t...
 * More info in: https://en.cppreference.com/w/cpp/language/template_argument_deduction
 *
 * The constexpr was added so the compiler knows that it can try to resolve this in compile time.
 * It is kind of like a macro (#define), with the difference that one can get extra compiler checks. Constexpr functions
 * can simply replace the function call by the content of the function (like a macro does), but the compiler also
 * checks:
 *      * if the parameter types are matching
 *      * if you would have access a non static address, it would actually not optimize it like a macro, because it
 * would result in undefined behavior. More info on constexpr can be found in
 * https://en.cppreference.com/w/cpp/language/constexpr
 *
 * One nice thing to check is looking at the disassembly (in CCS View->Disassembly) with and without the constexpr, just
 * to get a feeling. You can see that many instructions are added when not having the constexpr specifier.
 */
template<typename TYPE>
constexpr void setRegisterBits(volatile TYPE& registerRef, TYPE bitSelection) noexcept {
  registerRef |= bitSelection;
}

/**
 * Method to set the desired bits of a register to 0.
 * @param registerRef is the reference to the register to be accessed. For info on reference check the documentation of
 * setRegister
 * @param bitSelection is the bits in which you want to set to 0. E.g if you want to reset bit4 of the register P1DIR
 * you would call:
 *                     @code
 *                        resetRegisterBits(P1DIR, 0x10);
 *                     @endcode
 * @tparam TYPE to enforce that one is setting the bitSelection according to the register size. More info bellow.
 *
 * For info on template or constexpr specifier, check the documentation of setRegister
 */
template<typename TYPE>
constexpr void resetRegisterBits(volatile TYPE& registerRef, TYPE bitSelection) noexcept {
  registerRef &= ~bitSelection;
}

/**
 * Method to toggle the desired bits of a register.
 * @param registerRef is the reference to the register to be accessed. For info on reference check the documentation of
 * setRegister
 * @param bitSelection is the bits in which you want to toggle. E.g if you want to toggle bit4 of the register P1DIR you
 * would call:
 *                     @code
 *                        toggleRegisterBits(P1DIR, 0x10);
 *                     @endcode
 * @tparam TYPE to enforce that one is setting the bitSelection according to the register size. More info bellow.
 * For info on template or constexpr specifier, check the documentation of setRegister
 */
template<typename TYPE>
constexpr void toggleRegisterBits(volatile TYPE& registerRef, TYPE bitSelection) noexcept {
  registerRef ^= bitSelection;
}

/**
 * Method to get specific bits from a register and shift them to the right.
 * @param registerRef is the reference to the register to be accessed. For info on reference check the documentation of
 * setRegister
 * @param bitSelection is the bits in which you want to get. E.g if you want to get the value of bit4 of the register
 * P1DIR and shift the result 4 times to the right, so the result would be 1 or 0, then you would call:
 *                     @code
 *                        getRegisterBits(P1DIR, 0x10, 4);
 *                     @endcode
 * @tparam TYPE to enforce that one is setting the bitSelection according to the register size. More info bellow.
 * For info on template or constexpr specifier, check the documentation of setRegister
 */
template<typename TYPE>
constexpr TYPE getRegisterBits(volatile TYPE& registerRef, TYPE bitSelection, TYPE shiftsRight) noexcept {
  return (registerRef & bitSelection) >> shiftsRight;
}

#endif  // MICROTECH_HELPERS_HPP

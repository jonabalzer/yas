/*
Copyright (c) 2011, Michael Kazhdan and Ming Chuang
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer. Redistributions in binary form must reproduce
the above copyright notice, this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution. 

Neither the name of the Johns Hopkins University nor the names of its contributors
may be used to endorse or promote products derived from this software without specific
prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

#ifndef ARRAY_INCLUDED
#define ARRAY_INCLUDED

#include <vector>

#define ARRAY_DEBUG 0
#ifdef _WIN64
#define ASSERT( x ) { if( !( x ) ) __debugbreak(); }
#else // !_WIN64
#ifdef _WIN32
#define ASSERT( x ) { if( !( x ) ) _asm{ int 0x03 } }
#else // !_WIN32
#define ASSERT( x ) { if( !( x ) ) exit(0); }
#endif // _WIN32
#endif // _WIN64

namespace PoissonRec {

// Code from http://stackoverflow.com
void* aligned_malloc( size_t size , size_t align )
{

	// Align enough for the data, the alignment padding, and room to store a pointer to the actual start of the memory
	void*  mem = malloc( size + align + sizeof( void* ) );

    // The position at which we could potentially start addressing
	char* amem = ( (char*)mem ) + sizeof( void* );

    // Add align-1 to the start of the address and then zero out at most of the first align-1 bits.
	amem = ( char* )( ( (size_t)( ( (char*)amem ) + (align-1) ) ) & ~( align-1 ) );

    // Pre-write the actual address
	( ( void** ) amem )[-1] = mem;

    return amem;

}

void aligned_free( void* mem ) { free( ( ( void** )mem )[-1] ); }

#define      Pointer( ... )       __VA_ARGS__*
#define ConstPointer( ... ) const __VA_ARGS__*

#define        FreePointer( ... ) { if( __VA_ARGS__ )         free( __VA_ARGS__ ) ,                   __VA_ARGS__ = NULL; }
#define AlignedFreePointer( ... ) { if( __VA_ARGS__ ) aligned_free( __VA_ARGS__ ) ,                   __VA_ARGS__ = NULL; }
#define      DeletePointer( ... ) { if( __VA_ARGS__ )      delete[] __VA_ARGS__ ,                     __VA_ARGS__ = NULL; }

template< class C > C*          NewPointer(        size_t size ,                    const char* name=NULL ){ return new C[size]; }
template< class C > C*        AllocPointer(        size_t size ,                    const char* name=NULL ){ return (C*)        malloc(        sizeof(C) * size             ); }
template< class C > C* AlignedAllocPointer(        size_t size , size_t alignment , const char* name=NULL ){ return (C*)aligned_malloc(        sizeof(C) * size , alignment ); }
template< class C > C*      ReAllocPointer( C* c , size_t size ,                    const char* name=NULL ){ return (C*)       realloc( c    , sizeof(C) * size             ); }

template< class C > C* NullPointer( void ){ return NULL; }

template< class C >       C* PointerAddress(       C* c ){ return c; }
template< class C > const C* PointerAddress( const C* c ){ return c; }
template< class C >       C* GetPointer(       C& c ){ return &c; }
template< class C > const C* GetPointer( const C& c ){ return &c; }
template< class C >       C* GetPointer(       std::vector< C >& v ){ return &v[0]; }
template< class C > const C* GetPointer( const std::vector< C >& v ){ return &v[0]; }

} // end of namespace

#endif // ARRAY_INCLUDED

#if defined(WICED)
#include "wiced_duktape_tests_api_internal.h"
#endif

/*===
*** test_getprop_a (duk_safe_call)
obj.foo -> rc=1, result='fooval'
obj.nonexistent -> rc=0, result='undefined'
obj[123] -> rc=1, result='123val'
arr.nonexistent -> rc=0, result='undefined'
arr[2] -> rc=1, result='quux'
arr.length -> rc=1, result='3'
'test_string'[5] -> rc=1, result='s'
'test_string'.length -> rc=1, result='11'
final top: 3
==> rc=0, result='undefined'
*** test_getprop_b (duk_safe_call)
==> rc=1, result='RangeError: invalid stack index 234'
*** test_getprop_c (duk_safe_call)
==> rc=1, result='RangeError: invalid stack index -2147483648'
*** test_getprop_d (duk_safe_call)
Math.PI is 3.141593
configuration setting present, value: setting value
final top: 4
==> rc=0, result='undefined'
*** test_getprop_e (duk_safe_call)
==> rc=1, result='TypeError: cannot read property 'foo' of null'
*** test_getpropstring_a (duk_safe_call)
obj.foo -> rc=1, result='fooval'
obj.foo -> rc=1, result='fooval'
obj.nonexistent -> rc=0, result='undefined'
obj['123'] -> rc=1, result='123val'
arr.nonexistent -> rc=0, result='undefined'
arr['2'] -> rc=1, result='quux'
arr.length -> rc=1, result='3'
'test_string'['5'] -> rc=1, result='s'
'test_string'.length -> rc=1, result='11'
final top: 3
==> rc=0, result='undefined'
*** test_getpropstring_b (duk_safe_call)
==> rc=1, result='RangeError: invalid stack index 234'
*** test_getpropstring_c (duk_safe_call)
==> rc=1, result='RangeError: invalid stack index -2147483648'
*** test_getpropindex_a (duk_safe_call)
obj[31337] -> rc=0, result='undefined'
obj[123] -> rc=1, result='123val'
arr[31337] -> rc=0, result='undefined'
arr[2] -> rc=1, result='quux'
'test_string'[5] -> rc=1, result='s'
final top: 3
==> rc=0, result='undefined'
*** test_getpropindex_b (duk_safe_call)
==> rc=1, result='RangeError: invalid stack index 234'
*** test_getpropindex_c (duk_safe_call)
==> rc=1, result='RangeError: invalid stack index -2147483648'
*** test_getproplstring_a (duk_safe_call)
obj.foo -> rc=1, result='fooval'
obj.nonexistent -> rc=0, result='undefined'
obj['123'] -> rc=1, result='123val'
obj['nul<NUL>key'] -> rc=1, result='nulval'
final top: 3
==> rc=0, result='undefined'
*** test_getproplstring_b (duk_safe_call)
==> rc=1, result='RangeError: invalid stack index 234'
*** test_getproplstring_c (duk_safe_call)
==> rc=1, result='RangeError: invalid stack index -2147483648'
===*/

static void prep(duk_context *ctx) {
	duk_set_top(ctx, 0);

	/* 0: object with both string and number keys */
	duk_push_string(ctx, "{\"foo\": \"fooval\", \"bar\": \"barval\", \"123\": \"123val\", \"nul\\u0000key\": \"nulval\"}");
	(void) duk_json_decode(ctx, -1);

	/* 1: array with 3 elements */
	duk_push_string(ctx, "[ \"foo\", \"bar\", \"quux\" ]");
	(void) duk_json_decode(ctx, -1);

	/* 2: plain string */
	duk_push_string(ctx, "test_string");
}

/* duk_get_prop(), success cases */
static duk_ret_t test_getprop_a(duk_context *ctx, void *udata) {
	duk_ret_t rc;

	(void) udata;

	prep(ctx);

	duk_push_string(ctx, "foo");
	rc = duk_get_prop(ctx, 0);
	printf("obj.foo -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	duk_push_string(ctx, "nonexistent");
	rc = duk_get_prop(ctx, 0);
	printf("obj.nonexistent -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	duk_push_int(ctx, 123);
	rc = duk_get_prop(ctx, 0);
	printf("obj[123] -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	duk_push_string(ctx, "nonexistent");
	rc = duk_get_prop(ctx, 1);
	printf("arr.nonexistent -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	duk_push_int(ctx, 2);
	rc = duk_get_prop(ctx, 1);
	printf("arr[2] -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	duk_push_string(ctx, "length");
	rc = duk_get_prop(ctx, 1);
	printf("arr.length -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	duk_push_int(ctx, 5);
	rc = duk_get_prop(ctx, 2);
	printf("'test_string'[5] -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	duk_push_string(ctx, "length");
	rc = duk_get_prop(ctx, 2);
	printf("'test_string'.length -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	printf("final top: %ld\n", (long) duk_get_top(ctx));
	return 0;
}

/* duk_get_prop(), invalid index */
static duk_ret_t test_getprop_b(duk_context *ctx, void *udata) {
	duk_ret_t rc;

	(void) udata;

	prep(ctx);

	duk_push_string(ctx, "foo");
	rc = duk_get_prop(ctx, 234);
	printf("obj.foo -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	printf("final top: %ld\n", (long) duk_get_top(ctx));
	return 0;
}

/* duk_get_prop(), DUK_INVALID_INDEX */
static duk_ret_t test_getprop_c(duk_context *ctx, void *udata) {
	duk_ret_t rc;

	(void) udata;

	prep(ctx);

	duk_push_string(ctx, "foo");
	rc = duk_get_prop(ctx, DUK_INVALID_INDEX);
	printf("obj.foo -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	printf("final top: %ld\n", (long) duk_get_top(ctx));
	return 0;
}

/* duk_get_prop(), test in API doc (more or less) */
static duk_ret_t test_getprop_d(duk_context *ctx, void *udata) {
	int cfg_idx;

	(void) udata;

	prep(ctx);

	/* reading [global object].Math.PI */
	duk_push_global_object(ctx);    /* -> [ global ] */
	duk_push_string(ctx, "Math");   /* -> [ global "Math" ] */
	duk_get_prop(ctx, -2);          /* -> [ global Math ] */
	duk_push_string(ctx, "PI");     /* -> [ global Math "PI" ] */
	duk_get_prop(ctx, -2);          /* -> [ global Math PI ] */
	printf("Math.PI is %lf\n", duk_get_number(ctx, -1));
	duk_pop_n(ctx, 3);

	/* fake config object */
	cfg_idx = duk_get_top(ctx);
	duk_push_string(ctx, "{\"mySetting\": \"setting value\"}");
	duk_json_decode(ctx, cfg_idx);

	/* reading a configuration value, cfg_idx is normalized
	 * index of a configuration object.
	 */
	duk_push_string(ctx, "mySetting");
	if (duk_get_prop(ctx, cfg_idx)) {
	    const char *str_value = duk_to_string(ctx, -1);
	    printf("configuration setting present, value: %s\n", str_value);
	} else {
	    printf("configuration setting missing\n");
	}
	duk_pop(ctx);  /* remember to pop, regardless of whether or not present */

	printf("final top: %ld\n", (long) duk_get_top(ctx));
	return 0;
}

/* duk_get_prop(), not object coercible */
static duk_ret_t test_getprop_e(duk_context *ctx, void *udata) {
	duk_ret_t rc;

	(void) udata;

	duk_set_top(ctx, 0);

	duk_push_null(ctx);
	duk_push_string(ctx, "foo");
	rc = duk_get_prop(ctx, -2);
	printf("null.foo -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	printf("final top: %ld\n", (long) duk_get_top(ctx));
	return 0;
}

/* duk_get_prop_string(), success cases */
static duk_ret_t test_getpropstring_a(duk_context *ctx, void *udata) {
	duk_ret_t rc;

	(void) udata;

	prep(ctx);

	rc = duk_get_prop_string(ctx, 0, "foo");
	printf("obj.foo -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	rc = duk_get_prop_string(ctx, 0, "foo" "\x00" "bar");  /* embedded NUL terminates key */
	printf("obj.foo -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	rc = duk_get_prop_string(ctx, 0, "nonexistent");
	printf("obj.nonexistent -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	rc = duk_get_prop_string(ctx, 0, "123");
	printf("obj['123'] -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	rc = duk_get_prop_string(ctx, 1, "nonexistent");
	printf("arr.nonexistent -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	rc = duk_get_prop_string(ctx, 1, "2");
	printf("arr['2'] -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	rc = duk_get_prop_string(ctx, 1, "length");
	printf("arr.length -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	rc = duk_get_prop_string(ctx, 2, "5");
	printf("'test_string'['5'] -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	rc = duk_get_prop_string(ctx, 2, "length");
	printf("'test_string'.length -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	printf("final top: %ld\n", (long) duk_get_top(ctx));
	return 0;
}

/* duk_get_prop_string(), invalid index */
static duk_ret_t test_getpropstring_b(duk_context *ctx, void *udata) {
	duk_ret_t rc;

	(void) udata;

	prep(ctx);

	rc = duk_get_prop_string(ctx, 234, "foo");
	printf("obj.foo -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	printf("final top: %ld\n", (long) duk_get_top(ctx));
	return 0;
}

/* duk_get_prop_string(), DUK_INVALID_INDEX */
static duk_ret_t test_getpropstring_c(duk_context *ctx, void *udata) {
	duk_ret_t rc;

	(void) udata;

	prep(ctx);

	rc = duk_get_prop_string(ctx, DUK_INVALID_INDEX, "foo");
	printf("obj.foo -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	printf("final top: %ld\n", (long) duk_get_top(ctx));
	return 0;
}

/* duk_get_prop_index(), success cases */
static duk_ret_t test_getpropindex_a(duk_context *ctx, void *udata) {
	duk_ret_t rc;

	(void) udata;

	prep(ctx);

	rc = duk_get_prop_index(ctx, 0, 31337);
	printf("obj[31337] -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	rc = duk_get_prop_index(ctx, 0, 123);
	printf("obj[123] -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	rc = duk_get_prop_index(ctx, 1, 31337);
	printf("arr[31337] -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	rc = duk_get_prop_index(ctx, 1, 2);
	printf("arr[2] -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	rc = duk_get_prop_index(ctx, 2, 5);
	printf("'test_string'[5] -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	printf("final top: %ld\n", (long) duk_get_top(ctx));
	return 0;
}

/* duk_get_prop_index(), invalid index */
static duk_ret_t test_getpropindex_b(duk_context *ctx, void *udata) {
	duk_ret_t rc;

	(void) udata;

	prep(ctx);

	rc = duk_get_prop_index(ctx, 234, 123);
	printf("obj[123] -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	printf("final top: %ld\n", (long) duk_get_top(ctx));
	return 0;
}

/* duk_get_prop_index(), DUK_INVALID_INDEX */
static duk_ret_t test_getpropindex_c(duk_context *ctx, void *udata) {
	duk_ret_t rc;

	(void) udata;

	prep(ctx);

	rc = duk_get_prop_index(ctx, DUK_INVALID_INDEX, 123);
	printf("obj[123] -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	printf("final top: %ld\n", (long) duk_get_top(ctx));
	return 0;
}

/* duk_get_prop_lstring(), success cases */
static duk_ret_t test_getproplstring_a(duk_context *ctx, void *udata) {
	duk_ret_t rc;

	(void) udata;

	prep(ctx);

	rc = duk_get_prop_lstring(ctx, 0, "foox", 3);
	printf("obj.foo -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	rc = duk_get_prop_lstring(ctx, 0, "nonexistent", 11);
	printf("obj.nonexistent -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	rc = duk_get_prop_lstring(ctx, 0, "123", 3);
	printf("obj['123'] -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	rc = duk_get_prop_lstring(ctx, 0, "nul" "\x00" "key", 7);
	printf("obj['nul<NUL>key'] -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	printf("final top: %ld\n", (long) duk_get_top(ctx));
	return 0;
}

/* duk_get_prop_lstring(), invalid index */
static duk_ret_t test_getproplstring_b(duk_context *ctx, void *udata) {
	duk_ret_t rc;

	(void) udata;

	prep(ctx);

	rc = duk_get_prop_lstring(ctx, 234, "foox", 3);
	printf("obj.foo -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	printf("final top: %ld\n", (long) duk_get_top(ctx));
	return 0;
}

/* duk_get_prop_lstring(), DUK_INVALID_INDEX */
static duk_ret_t test_getproplstring_c(duk_context *ctx, void *udata) {
	duk_ret_t rc;

	(void) udata;

	prep(ctx);

	rc = duk_get_prop_lstring(ctx, DUK_INVALID_INDEX, "foox", 3);
	printf("obj.foo -> rc=%d, result='%s'\n", (int) rc, duk_to_string(ctx, -1));
	duk_pop(ctx);

	printf("final top: %ld\n", (long) duk_get_top(ctx));
	return 0;
}

#if defined(WICED)
void wiced_duktape_tests_api_get_prop(duk_context *ctx) {
#else
void test(duk_context *ctx) {
#endif
	TEST_SAFE_CALL(test_getprop_a);
	TEST_SAFE_CALL(test_getprop_b);
	TEST_SAFE_CALL(test_getprop_c);
	TEST_SAFE_CALL(test_getprop_d);
	TEST_SAFE_CALL(test_getprop_e);

	TEST_SAFE_CALL(test_getpropstring_a);
	TEST_SAFE_CALL(test_getpropstring_b);
	TEST_SAFE_CALL(test_getpropstring_c);

	TEST_SAFE_CALL(test_getpropindex_a);
	TEST_SAFE_CALL(test_getpropindex_b);
	TEST_SAFE_CALL(test_getpropindex_c);

	TEST_SAFE_CALL(test_getproplstring_a);
	TEST_SAFE_CALL(test_getproplstring_b);
	TEST_SAFE_CALL(test_getproplstring_c);
}

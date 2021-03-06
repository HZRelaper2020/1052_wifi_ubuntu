#if defined(WICED)
#include "wiced_duktape_tests_api_internal.h"
#endif

/*===
*** test_1 (duk_safe_call)
top: 18
index 0, is-null: 1
index 1, is-null: 1
index 2, is-null: 1
index 3, is-null: 1
index 4, is-null: 1
index 5, is-null: 1
index 6, is-null: 1
index 7, is-null: 1
index 8, is-null: 1
index 9, is-null: 1
index 10, is-null: 1
index 11, is-null: 1
index 12, is-null: 1
index 13, is-null: 1
index 14, is-null: 1
index 15, is-null: 1
index 16, is-null: 1
index 17, is-null: 1
==> rc=0, result='undefined'
*** test_2 (duk_safe_call)
==> rc=1, result='RangeError: invalid stack index 3'
*** test_3 (duk_safe_call)
==> rc=1, result='RangeError: invalid stack index -2147483648'
===*/

static duk_ret_t test_1(duk_context *ctx, void *udata) {
	duk_idx_t i, n;

	(void) udata;

	duk_set_top(ctx, 0);
	duk_push_undefined(ctx);
	duk_push_null(ctx);
	duk_push_true(ctx);
	duk_push_false(ctx);
	duk_push_int(ctx, 0);
	duk_push_int(ctx, 1);
	duk_push_nan(ctx);
	duk_push_number(ctx, INFINITY);
	duk_push_string(ctx, "");
	duk_push_string(ctx, "foo");
	duk_push_object(ctx);
	duk_push_thread(ctx);
	duk_push_fixed_buffer(ctx, 0);
	duk_push_fixed_buffer(ctx, 1024);
	duk_push_dynamic_buffer(ctx, 0);
	duk_push_dynamic_buffer(ctx, 1024);
	duk_push_pointer(ctx, (void *) NULL);
	duk_push_pointer(ctx, (void *) 0xdeadbeef);

	n = duk_get_top(ctx);
	printf("top: %ld\n", (long) n);
	for (i = 0; i < n; i++) {
		duk_to_null(ctx, i);
		printf("index %ld, is-null: %d\n", (long) i, (int) duk_is_null(ctx, i));
	}

	return 0;
}

static duk_ret_t test_2(duk_context *ctx, void *udata) {
	(void) udata;

	duk_set_top(ctx, 0);
	duk_to_null(ctx, 3);
	printf("index 3 OK\n");
	return 0;
}

static duk_ret_t test_3(duk_context *ctx, void *udata) {
	(void) udata;

	duk_set_top(ctx, 0);
	duk_to_null(ctx, DUK_INVALID_INDEX);
	printf("index DUK_INVALID_INDEX OK\n");
	return 0;
}

#if defined(WICED)
void wiced_duktape_tests_api_to_null(duk_context *ctx) {
#else
void test(duk_context *ctx) {
#endif
	TEST_SAFE_CALL(test_1);
	TEST_SAFE_CALL(test_2);
	TEST_SAFE_CALL(test_3);
}
